#!/usr/bin/env python3
"""Convert the 46-dim height policy ONNX file into a TensorRT engine.

Runtime contract:
  - input tensor name:  obs
  - output tensor name: action
  - input shape:        [1, 276]  (6 steps x 46 dims)
  - output shape:       [12] or [1, 12]
  - I/O dtype:          float32
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Optional


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ONNX = REPO_ROOT / "models/policy_2_blind_terrain/policy701.onnx"
DEFAULT_ENGINE = REPO_ROOT / "models/policy_2_blind_terrain/policy701.engine"
DEFAULT_TRTEXEC = "/usr/src/tensorrt/bin/trtexec"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert the 46-dim height policy ONNX file to a TensorRT engine."
    )
    parser.add_argument(
        "--input",
        default=str(DEFAULT_ONNX),
        help="Input ONNX file.",
    )
    parser.add_argument(
        "--output",
        default=str(DEFAULT_ENGINE),
        help="Output TensorRT engine file.",
    )
    parser.add_argument(
        "--trtexec",
        default=DEFAULT_TRTEXEC,
        help="Path to trtexec. If empty, the script will try PATH first.",
    )
    parser.add_argument(
        "--input-name",
        default="obs",
        help="Expected ONNX input tensor name.",
    )
    parser.add_argument(
        "--output-name",
        default="action",
        help="Expected ONNX output tensor name.",
    )
    parser.add_argument(
        "--obs-dim",
        type=int,
        default=276,
        help="Expected flattened observation dimension: 6 x 46 = 276.",
    )
    parser.add_argument(
        "--action-dim",
        type=int,
        default=12,
        help="Expected action dimension.",
    )
    parser.add_argument(
        "--workspace-mib",
        type=int,
        default=1024,
        help="TensorRT builder workspace in MiB.",
    )
    parser.add_argument(
        "--fp16",
        action="store_true",
        help="Allow TensorRT FP16 tactics. I/O bindings must still stay float32 for rl_controller.",
    )
    parser.add_argument(
        "--skip-onnx-check",
        action="store_true",
        help="Skip the optional ONNX graph validation step.",
    )
    parser.add_argument(
        "--skip-engine-check",
        action="store_true",
        help="Skip the optional TensorRT engine validation step.",
    )
    return parser.parse_args()


def fail(message: str) -> None:
    print(f"[ERROR] {message}", file=sys.stderr)
    raise SystemExit(1)


def warn(message: str) -> None:
    print(f"[WARN] {message}", file=sys.stderr)


def resolve_trtexec(path: str) -> str:
    if path.strip():
        candidate = Path(path).expanduser()
        if candidate.is_file():
            return str(candidate.resolve())
        found = shutil.which(path)
        if found:
            return found
        fail(f"trtexec not found: {path}")

    found = shutil.which("trtexec")
    if found:
        return found
    if Path(DEFAULT_TRTEXEC).is_file():
        return DEFAULT_TRTEXEC
    fail("trtexec not found in PATH or under /usr/src/tensorrt/bin/trtexec")


def load_onnx_optional(
    input_path: Path,
    expected_input: str,
    expected_output: str,
    obs_dim: int,
    action_dim: int,
) -> None:
    try:
        import onnx  # type: ignore
        from onnx import TensorProto  # type: ignore
    except ModuleNotFoundError:
        warn("Python package 'onnx' is not installed; skipping ONNX graph validation.")
        return

    model = onnx.load(str(input_path))
    inputs = list(model.graph.input)
    outputs = list(model.graph.output)

    if len(inputs) != 1 or len(outputs) != 1:
        fail(
            f"ONNX graph must have exactly one input and one output, got "
            f"{len(inputs)} input(s) and {len(outputs)} output(s)."
        )

    input_info = inputs[0]
    output_info = outputs[0]

    if input_info.name != expected_input:
        fail(f"ONNX input name is '{input_info.name}', expected '{expected_input}'.")
    if output_info.name != expected_output:
        fail(f"ONNX output name is '{output_info.name}', expected '{expected_output}'.")

    if input_info.type.tensor_type.elem_type != TensorProto.FLOAT:
        fail("ONNX input dtype must be float32.")
    if output_info.type.tensor_type.elem_type != TensorProto.FLOAT:
        fail("ONNX output dtype must be float32.")

    input_dims = [
        d.dim_value if d.dim_value > 0 else (-1 if d.dim_param else 0)
        for d in input_info.type.tensor_type.shape.dim
    ]
    output_dims = [
        d.dim_value if d.dim_value > 0 else (-1 if d.dim_param else 0)
        for d in output_info.type.tensor_type.shape.dim
    ]

    if len(input_dims) != 2 or input_dims[1] != obs_dim or input_dims[0] not in (1, -1):
        fail(f"ONNX input shape must be [1, {obs_dim}] or [-1, {obs_dim}], got {input_dims}.")

    if len(output_dims) == 1:
        if output_dims[0] != action_dim:
            fail(f"ONNX output shape must be [12] or [1, 12], got {output_dims}.")
    elif len(output_dims) == 2:
        if output_dims[1] != action_dim or output_dims[0] not in (1, -1):
            fail(f"ONNX output shape must be [12] or [1, 12], got {output_dims}.")
    else:
        fail(f"ONNX output rank must be 1 or 2, got {output_dims}.")


def infer_onnx_dynamic_batch(input_path: Path, expected_input: str) -> Optional[bool]:
    try:
        import onnx  # type: ignore
    except ModuleNotFoundError:
        return None

    model = onnx.load(str(input_path))
    inputs = list(model.graph.input)
    if len(inputs) != 1:
        return None
    input_info = inputs[0]
    if input_info.name != expected_input:
        return None

    dims = input_info.type.tensor_type.shape.dim
    if len(dims) != 2:
        return None

    first_dim = dims[0]
    if first_dim.dim_param:
        return True
    if first_dim.dim_value > 0:
        return False
    return None


def run_trtexec_build(
    trtexec: str,
    input_path: Path,
    output_path: Path,
    input_name: str,
    obs_dim: int,
    workspace_mib: int,
    fp16: bool,
    use_explicit_shapes: bool,
) -> None:
    cmd = [
        trtexec,
        f"--onnx={input_path}",
        f"--saveEngine={output_path}",
        f"--workspace={workspace_mib}",
        "--buildOnly",
    ]
    if use_explicit_shapes:
        shape_spec = f"{input_name}:1x{obs_dim}"
        cmd.extend([
            f"--shapes={shape_spec}",
            f"--minShapes={shape_spec}",
            f"--optShapes={shape_spec}",
            f"--maxShapes={shape_spec}",
        ])
    if fp16:
        cmd.append("--fp16")

    print("[INFO] Running:", " ".join(str(part) for part in cmd))
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as exc:
        fail(
            f"trtexec failed with return code {exc.returncode}. "
            "If the log shows NvRmMemInit/Cuda operation not supported, run this on the Jetson "
            "with CUDA/GPU access available, not inside a restricted environment."
        )


def validate_engine_optional(output_path: Path, obs_dim: int, action_dim: int) -> None:
    try:
        import tensorrt as trt  # type: ignore
    except Exception as exc:
        warn(f"TensorRT Python bindings are unavailable; skipping engine validation ({exc}).")
        return

    logger = trt.Logger(trt.Logger.WARNING)
    try:
        with open(output_path, "rb") as f:
            engine_bytes = f.read()
        runtime = trt.Runtime(logger)
        engine = runtime.deserialize_cuda_engine(engine_bytes)
    except Exception as exc:
        warn(f"Failed to deserialize the built engine for validation: {exc}")
        return

    if engine is None:
        warn("TensorRT returned a null engine during validation.")
        return

    if hasattr(engine, "num_bindings"):
        if engine.num_bindings != 2:
            fail(f"Engine must expose exactly 2 bindings, got {engine.num_bindings}.")
        binding_count = engine.num_bindings
        for idx in range(binding_count):
            name = engine.get_binding_name(idx)
            is_input = engine.binding_is_input(idx)
            dtype = engine.get_binding_dtype(idx)
            shape = tuple(engine.get_binding_shape(idx))
            if dtype != trt.DataType.FLOAT:
                fail(f"Engine binding '{name}' must be float32, got {dtype}.")
            if is_input:
                if len(shape) != 2 or shape[1] != obs_dim or shape[0] not in (1, -1):
                    fail(f"Engine input binding '{name}' has unexpected shape {shape}.")
            else:
                if len(shape) == 1:
                    if shape[0] != action_dim:
                        fail(f"Engine output binding '{name}' has unexpected shape {shape}.")
                elif len(shape) == 2:
                    if shape[1] != action_dim or shape[0] not in (1, -1):
                        fail(f"Engine output binding '{name}' has unexpected shape {shape}.")
                else:
                    fail(f"Engine output binding '{name}' has unexpected shape {shape}.")
    else:
        warn(
            "TensorRT Python API does not expose legacy binding inspection helpers; "
            "skipping detailed validation."
        )


def main() -> int:
    args = parse_args()
    input_path = Path(args.input).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    if not input_path.is_file():
        fail(f"Input ONNX file not found: {input_path}")

    output_path.parent.mkdir(parents=True, exist_ok=True)

    if not args.skip_onnx_check:
        load_onnx_optional(
            input_path,
            args.input_name,
            args.output_name,
            args.obs_dim,
            args.action_dim,
        )
    dynamic_batch = infer_onnx_dynamic_batch(input_path, args.input_name)

    trtexec = resolve_trtexec(args.trtexec)
    run_trtexec_build(
        trtexec,
        input_path,
        output_path,
        args.input_name,
        args.obs_dim,
        args.workspace_mib,
        args.fp16,
        dynamic_batch is True,
    )

    if not args.skip_engine_check:
        validate_engine_optional(output_path, args.obs_dim, args.action_dim)

    print("[OK] 46-dim TensorRT engine export finished")
    print(f"input_onnx : {input_path}")
    print(f"output_trt : {output_path}")
    print(f"input_name : {args.input_name}")
    print(f"output_name: {args.output_name}")
    print(f"obs_shape  : [1, {args.obs_dim}]")
    print(f"act_shape  : [12] or [1, 12]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
