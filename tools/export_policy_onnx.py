#!/usr/bin/env python3
"""Export the project policy to ONNX using the current runtime contract.

This script is intentionally aligned with the current rl_controller node:
  - input tensor name:  obs
  - output tensor name: action
  - input shape:        [1, 270]  (6 steps x 45 dims)
  - output shape:       [12] or [1, 12]

Not directly supported:
  - training checkpoints like model_3000.pt that only contain state_dict
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import torch


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = REPO_ROOT / "policy.pt"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export the project RL policy (.pt) to ONNX."
    )
    parser.add_argument(
        "--input",
        default=str(DEFAULT_INPUT),
        help="Input .pt file. Prefer the scripted inference policy.pt from this repo.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help=(
            "Output ONNX path. If omitted, use <input_basename>.onnx next to the input "
            "file when writable; otherwise fall back to the current working directory."
        ),
    )
    parser.add_argument(
        "--obs-dim",
        type=int,
        default=270,
        help="Observation history dimension. Current runtime expects 270.",
    )
    parser.add_argument(
        "--action-dim",
        type=int,
        default=12,
        help="Action dimension. Current runtime expects 12.",
    )
    parser.add_argument(
        "--input-name",
        default="obs",
        help="ONNX input tensor name used by the current project.",
    )
    parser.add_argument(
        "--output-name",
        default="action",
        help="ONNX output tensor name used by the current project.",
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
                "Please export or use the play-time scripted 'policy.pt' instead."
            )
        fail(
            f"'{path}' is not a supported scripted policy file. Loaded object type: {type(obj)}"
        )

    if not hasattr(model, "forward"):
        fail(f"'{path}' does not expose a callable forward(obs_history).")
    return model


def validate_output_contract(output: torch.Tensor, action_dim: int) -> None:
    if output.ndim == 1:
        if output.numel() != action_dim:
            fail(
                f"Model output has {output.numel()} elements, expected {action_dim}."
            )
        return

    if output.ndim == 2:
        if output.shape[0] != 1 or output.shape[1] != action_dim:
            fail(
                f"Model output shape is {tuple(output.shape)}, expected [12] or [1, 12]."
            )
        return

    fail(
        f"Unexpected output rank {output.ndim}. Expected [12] or [1, 12]-style action output."
    )


def resolve_output_path(input_path: Path, output_arg: str | None) -> Path:
    if output_arg:
        return Path(output_arg).expanduser().resolve()

    preferred = input_path.with_suffix(".onnx")
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

    model = load_scripted_policy(input_path)
    model.eval()

    dummy_input = torch.zeros(1, args.obs_dim, dtype=torch.float32)

    try:
        with torch.no_grad():
            dummy_output = model(dummy_input)
    except Exception as exc:
        fail(
            f"Forward pass failed with dummy input shape (1, {args.obs_dim}). "
            f"Check obs_dim. Original error: {exc}"
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
            fail(
                "Python package 'onnx' is not installed in the current environment. "
                "Install it first with: pip install onnx"
            )
        raise
    except Exception as exc:
        fail(f"ONNX export failed: {exc}")

    print("[OK] Export finished")
    print(f"input_pt   : {input_path}")
    print(f"output_onnx: {output_path}")
    print(f"obs_shape  : {tuple(dummy_input.shape)}")
    print(f"act_shape  : {tuple(dummy_output.shape)}")
    print(f"input_name : {args.input_name}")
    print(f"output_name: {args.output_name}")
    print(f"opset      : {args.opset}")
    print(f"dynamic    : {args.dynamic_batch}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
