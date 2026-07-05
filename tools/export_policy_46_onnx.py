#!/usr/bin/env python3
"""Export a 46-dim single-step HIM policy to ONNX.

This handles older scripted PolicyExporterHIM wrappers that still call:
    actor(cat(obs_history[:, 0:45], vel, z))

For the height-command model the correct actor input is:
    actor(cat(obs_history[:, 0:46], vel, z))

The estimator still consumes the full history tensor:
    6 history steps x 46 dims = 276 dims
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import torch
import torch.nn.functional as F


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = REPO_ROOT / "models/policy_2_blind_terrain/policy.pt"


class HeightPolicy46Wrapper(torch.nn.Module):
    def __init__(
        self,
        estimator: torch.nn.Module,
        actor: torch.nn.Module,
        one_step_obs: int,
        estimator_out_dim: int,
    ):
        super().__init__()
        self.estimator = estimator
        self.actor = actor
        self.one_step_obs = one_step_obs
        self.estimator_out_dim = estimator_out_dim

    def forward(self, obs_history: torch.Tensor) -> torch.Tensor:
        parts = self.estimator(obs_history)[:, : self.estimator_out_dim]
        vel = parts[..., :3]
        z = F.normalize(parts[..., 3:], dim=-1, p=2.0)
        actor_obs = obs_history[:, : self.one_step_obs]
        return self.actor(torch.cat((actor_obs, vel, z), dim=1))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export the 46-dim height-command RL policy (.pt) to ONNX."
    )
    parser.add_argument(
        "--input",
        default=str(DEFAULT_INPUT),
        help="Input scripted policy.pt file.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help=(
            "Output ONNX path. If omitted, use <input_basename>_46.onnx next to "
            "the input file when writable; otherwise fall back to the current directory."
        ),
    )
    parser.add_argument(
        "--history-steps",
        type=int,
        default=6,
        help="Observation history length. Height policy runtime uses 6.",
    )
    parser.add_argument(
        "--one-step-obs",
        type=int,
        default=46,
        help="Single-step observation dimension. Height policy uses 46.",
    )
    parser.add_argument(
        "--obs-dim",
        type=int,
        default=None,
        help="Flattened history dimension. Defaults to history_steps * one_step_obs = 276.",
    )
    parser.add_argument(
        "--estimator-out-dim",
        type=int,
        default=19,
        help="Estimator output slice width: vel(3) + latent z(16) = 19.",
    )
    parser.add_argument(
        "--action-dim",
        type=int,
        default=12,
        help="Action dimension. Runtime expects 12.",
    )
    parser.add_argument(
        "--input-name",
        default="obs",
        help="ONNX input tensor name used by rl_controller.",
    )
    parser.add_argument(
        "--output-name",
        default="action",
        help="ONNX output tensor name used by rl_controller.",
    )
    parser.add_argument(
        "--opset",
        type=int,
        default=13,
        help="ONNX opset version. TensorRT 8.5.x usually works well with 13.",
    )
    parser.add_argument(
        "--dynamic-batch",
        action="store_true",
        help="Export batch dimension as dynamic. Default is fixed batch=1.",
    )
    return parser.parse_args()


def fail(message: str) -> None:
    print(f"[ERROR] {message}", file=sys.stderr)
    raise SystemExit(1)


def load_scripted_policy(path: Path) -> torch.jit.ScriptModule:
    try:
        model = torch.jit.load(str(path), map_location="cpu")
    except Exception:
        try:
            obj = torch.load(str(path), map_location="cpu")
        except Exception as exc:
            fail(f"Unable to load '{path}': {exc}")

        if isinstance(obj, dict):
            keys = ", ".join(list(obj.keys())[:8])
            fail(
                f"'{path}' looks like a training checkpoint/state_dict, not a scripted "
                f"inference policy. Sample keys: [{keys}]. "
                "Please export or use the play-time scripted policy.pt instead."
            )
        fail(
            f"'{path}' is not a supported scripted policy file. Loaded object type: {type(obj)}"
        )

    if not hasattr(model, "forward"):
        fail(f"'{path}' does not expose a callable forward(obs_history).")
    if not hasattr(model, "estimator") or not hasattr(model, "actor"):
        fail(
            f"'{path}' does not expose estimator and actor. "
            "This 46-dim wrapper exporter is only for PolicyExporterHIM-style scripted policies."
        )
    return model


def clone_scripted_sequential(module: torch.nn.Module, module_name: str) -> torch.nn.Sequential:
    layers = []
    for name, child in module.named_children():
        if hasattr(child, "weight") and hasattr(child, "bias"):
            weight = child.weight.detach().cpu()
            bias = child.bias.detach().cpu()
            linear = torch.nn.Linear(weight.shape[1], weight.shape[0])
            with torch.no_grad():
                linear.weight.copy_(weight)
                linear.bias.copy_(bias)
            layers.append(linear)
        else:
            child_text = repr(child)
            if "ELU" in child_text:
                layers.append(torch.nn.ELU())
            else:
                fail(
                    f"Unsupported layer in {module_name}.{name}: {child_text}. "
                    "This exporter currently supports Linear + ELU Sequential policies."
                )

    if not layers:
        fail(f"{module_name} has no cloneable layers.")
    return torch.nn.Sequential(*layers)


def validate_output_contract(output: torch.Tensor, action_dim: int) -> None:
    if output.ndim == 1:
        if output.numel() != action_dim:
            fail(f"Model output has {output.numel()} elements, expected {action_dim}.")
        return

    if output.ndim == 2:
        if output.shape[0] != 1 or output.shape[1] != action_dim:
            fail(f"Model output shape is {tuple(output.shape)}, expected [12] or [1, 12].")
        return

    fail(f"Unexpected output rank {output.ndim}. Expected [12] or [1, 12].")


def resolve_output_path(input_path: Path, output_arg: str | None) -> Path:
    if output_arg:
        return Path(output_arg).expanduser().resolve()

    preferred = input_path.with_name(input_path.stem + "_46.onnx")
    try:
        preferred.parent.mkdir(parents=True, exist_ok=True)
        test_path = preferred.parent / ".onnx_write_test"
        with open(test_path, "w", encoding="utf-8"):
            pass
        test_path.unlink()
        return preferred
    except OSError:
        fallback = Path.cwd() / preferred.name
        return fallback.resolve()


def main() -> int:
    args = parse_args()
    input_path = Path(args.input).expanduser().resolve()
    output_path = resolve_output_path(input_path, args.output)

    if not input_path.is_file():
        fail(f"Input file not found: {input_path}")
    if args.history_steps <= 0 or args.one_step_obs <= 0:
        fail("history_steps and one_step_obs must be positive.")

    obs_dim = args.obs_dim
    if obs_dim is None:
        obs_dim = args.history_steps * args.one_step_obs
    if obs_dim != args.history_steps * args.one_step_obs:
        fail(
            f"obs_dim must equal history_steps * one_step_obs, got {obs_dim} vs "
            f"{args.history_steps} * {args.one_step_obs}."
        )

    scripted_model = load_scripted_policy(input_path)
    estimator = clone_scripted_sequential(scripted_model.estimator, "estimator")
    actor = clone_scripted_sequential(scripted_model.actor, "actor")
    model = HeightPolicy46Wrapper(
        estimator,
        actor,
        one_step_obs=args.one_step_obs,
        estimator_out_dim=args.estimator_out_dim,
    )
    model.eval()

    dummy_input = torch.zeros(1, obs_dim, dtype=torch.float32)

    try:
        with torch.no_grad():
            dummy_output = model(dummy_input)
    except Exception as exc:
        fail(
            f"Forward pass failed with wrapped 46-dim input shape (1, {obs_dim}). "
            f"Original error: {exc}"
        )

    if not isinstance(dummy_output, torch.Tensor):
        fail(f"Model forward output is not a Tensor: {type(dummy_output)}")

    validate_output_contract(dummy_output, args.action_dim)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    dynamic_axes = None
    if args.dynamic_batch:
        dynamic_axes = {
            args.input_name: {0: "batch"},
            args.output_name: {0: "batch"},
        }

    try:
        torch.onnx.export(
            model,
            dummy_input,
            str(output_path),
            export_params=True,
            opset_version=args.opset,
            do_constant_folding=True,
            input_names=[args.input_name],
            output_names=[args.output_name],
            dynamic_axes=dynamic_axes,
        )
    except ModuleNotFoundError as exc:
        if "onnx" in str(exc).lower():
            fail("Python package 'onnx' is not installed. Install it first with: pip install onnx")
        raise
    except Exception as exc:
        fail(f"ONNX export failed: {exc}")

    print("[OK] 46-dim policy ONNX export finished")
    print(f"input_pt    : {input_path}")
    print(f"output_onnx : {output_path}")
    print(f"obs_shape   : {tuple(dummy_input.shape)}")
    print(f"act_shape   : {tuple(dummy_output.shape)}")
    print(f"one_step_obs: {args.one_step_obs}")
    print(f"history     : {args.history_steps}")
    print(f"input_name  : {args.input_name}")
    print(f"output_name : {args.output_name}")
    print(f"opset       : {args.opset}")
    print(f"dynamic     : {args.dynamic_batch}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
