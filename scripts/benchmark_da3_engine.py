#!/home/ubuntu/bl/miniconda3/envs/robotdog/bin/python
"""Run DA3 TensorRT engine on a video and save side-by-side depth visualization.

Output:
left  = original video
right = colorized depth estimation
"""

import argparse
import os
import statistics
import time
from pathlib import Path

# Pin the host's physical second GPU before importing CUDA libraries.
os.environ["CUDA_VISIBLE_DEVICES"] = "1"

import cv2
import numpy as np
import tensorrt as trt
import torch
import torch.nn.functional as F


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ENGINE = PROJECT_ROOT / "models/da3/DA3METRIC-LARGE.fp16-batch1.engine"
DEFAULT_VIDEO = PROJECT_ROOT / "data/videos/test_video.mp4"


def torch_dtype(trt_dtype):
    mapping = {
        np.dtype(np.float32): torch.float32,
        np.dtype(np.float16): torch.float16,
        np.dtype(np.int32): torch.int32,
        np.dtype(np.int8): torch.int8,
        np.dtype(np.bool_): torch.bool,
    }

    dtype = np.dtype(trt.nptype(trt_dtype))

    if dtype not in mapping:
        raise TypeError(f"Unsupported TensorRT dtype: {trt_dtype}")

    return mapping[dtype]


def percentile(values, q):
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def preprocess_frame(frame_bgr, input_tensor, device, normalize=True, rgb=True):
    """
    OpenCV frame:
        HWC, uint8, BGR

    TensorRT input:
        usually shape = (1, 3, 280, 504)

    This function writes into input_tensor in-place.
    """

    if rgb:
        frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    else:
        frame = frame_bgr

    frame_tensor = torch.from_numpy(frame).to(device=device)

    # HWC -> CHW -> NCHW
    frame_tensor = frame_tensor.permute(2, 0, 1).contiguous()
    frame_tensor = frame_tensor.unsqueeze(0)

    frame_tensor = frame_tensor.to(dtype=input_tensor.dtype)

    if normalize:
        frame_tensor = frame_tensor / 255.0
        frame_tensor[:, 0].sub_(0.485).div_(0.229)
        frame_tensor[:, 1].sub_(0.456).div_(0.224)
        frame_tensor[:, 2].sub_(0.406).div_(0.225)

    target_h, target_w = input_tensor.shape[-2:]

    if frame_tensor.shape[-2:] != (target_h, target_w):
        frame_tensor = F.interpolate(
            frame_tensor,
            size=(target_h, target_w),
            mode="bilinear",
            align_corners=False,
        )

    if frame_tensor.shape != input_tensor.shape:
        raise RuntimeError(
            f"Preprocessed frame shape mismatch: got {tuple(frame_tensor.shape)}, "
            f"expected {tuple(input_tensor.shape)}"
        )

    input_tensor.copy_(frame_tensor)


def colorize_depth(depth, out_w, out_h, invert=False):
    """
    Convert raw depth tensor output to a color image.

    depth:
        numpy array, shape can be (1,1,H,W), (1,H,W), or (H,W)

    Returns:
        BGR uint8 image, shape = (out_h, out_w, 3)
    """

    depth = np.asarray(depth)

    while depth.ndim > 2:
        depth = depth[0]

    depth = depth.astype(np.float32)

    # Robust normalization per frame.
    # This usually makes visualization much easier to see than min/max if there are outliers.
    valid = np.isfinite(depth)

    if not np.any(valid):
        depth_u8 = np.zeros_like(depth, dtype=np.uint8)
    else:
        d = depth[valid]
        lo = np.percentile(d, 2)
        hi = np.percentile(d, 98)

        if hi - lo < 1e-6:
            depth_u8 = np.zeros_like(depth, dtype=np.uint8)
        else:
            depth_norm = (depth - lo) / (hi - lo)
            depth_norm = np.clip(depth_norm, 0.0, 1.0)

            if invert:
                depth_norm = 1.0 - depth_norm

            depth_u8 = (depth_norm * 255.0).astype(np.uint8)

    depth_u8 = cv2.resize(depth_u8, (out_w, out_h), interpolation=cv2.INTER_CUBIC)
    depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)

    return depth_color


def make_side_by_side(frame_bgr, depth_color, frame_index=None, fps_text=None):
    """
    Concatenate original frame and depth visualization.
    """

    h, w = frame_bgr.shape[:2]

    if depth_color.shape[:2] != (h, w):
        depth_color = cv2.resize(depth_color, (w, h), interpolation=cv2.INTER_LINEAR)

    left = frame_bgr.copy()
    right = depth_color.copy()

    cv2.putText(
        left,
        "Original",
        (30, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.6,
        (255, 255, 255),
        3,
        cv2.LINE_AA,
    )

    cv2.putText(
        right,
        "DA3 Depth",
        (30, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.6,
        (255, 255, 255),
        3,
        cv2.LINE_AA,
    )

    if frame_index is not None:
        text = f"Frame: {frame_index}"
        cv2.putText(
            left,
            text,
            (30, h - 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.1,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    if fps_text:
        cv2.putText(
            right,
            fps_text,
            (30, h - 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.1,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    return np.concatenate([left, right], axis=1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("--engine", type=Path, default=DEFAULT_ENGINE)
    parser.add_argument("--video", type=Path, default=DEFAULT_VIDEO)

    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output video path. Default: same directory as input video.",
    )

    parser.add_argument("--warmup", type=int, default=20)
    parser.add_argument("--max-frames", type=int, default=0, help="0 means process full video")

    parser.add_argument(
        "--no-normalize",
        action="store_true",
        help="Disable DA3 ImageNet normalization. Only useful for controlled experiments.",
    )

    parser.add_argument(
        "--bgr",
        action="store_true",
        help="Keep OpenCV BGR order instead of converting to RGB.",
    )

    parser.add_argument(
        "--invert-depth",
        action="store_true",
        help="Invert depth visualization colors. Use this if near/far looks visually reversed.",
    )

    parser.add_argument(
        "--codec",
        type=str,
        default="mp4v",
        help="VideoWriter codec. Default: mp4v",
    )

    args = parser.parse_args()

    if args.warmup < 0:
        parser.error("warmup must be >= 0")

    if args.max_frames < 0:
        parser.error("max-frames must be >= 0")

    if not args.engine.is_file():
        parser.error(f"engine not found: {args.engine}")

    if not args.video.is_file():
        parser.error(f"video not found: {args.video}")

    if args.output is None:
        args.output = args.video.with_name(args.video.stem + "_da3_depth_side_by_side.mp4")

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable in this environment")

    device = 0
    torch.cuda.set_device(device)

    logger = trt.Logger(trt.Logger.WARNING)
    runtime = trt.Runtime(logger)

    print(f"Loading engine: {args.engine}")
    load_start = time.perf_counter()

    engine = runtime.deserialize_cuda_engine(args.engine.read_bytes())

    if engine is None:
        raise RuntimeError("Failed to deserialize TensorRT engine")

    context = engine.create_execution_context()

    if context is None:
        raise RuntimeError("Failed to create TensorRT execution context")

    tensors = {}
    input_names = []
    output_names = []

    for index in range(engine.num_io_tensors):
        name = engine.get_tensor_name(index)
        mode = engine.get_tensor_mode(name)
        shape = tuple(engine.get_tensor_shape(name))

        if any(dim < 0 for dim in shape):
            if mode != trt.TensorIOMode.INPUT:
                raise RuntimeError(
                    f"Dynamic output shape is not handled in this script: {name}, shape={shape}"
                )

            profile_shape = engine.get_tensor_profile_shape(name, 0)[1]
            context.set_input_shape(name, profile_shape)
            shape = tuple(profile_shape)

        dtype = torch_dtype(engine.get_tensor_dtype(name))
        tensor = torch.empty(shape, dtype=dtype, device=f"cuda:{device}")

        if mode == trt.TensorIOMode.INPUT:
            input_names.append(name)
        else:
            output_names.append(name)

        context.set_tensor_address(name, tensor.data_ptr())
        tensors[name] = tensor

    if len(input_names) != 1:
        raise RuntimeError(f"Expected exactly 1 input tensor, got {input_names}")

    input_name = input_names[0]

    # Prefer output named "depth".
    if "depth" in output_names:
        depth_name = "depth"
    else:
        depth_name = output_names[0]

    stream = torch.cuda.Stream(device=device)

    cap = cv2.VideoCapture(str(args.video))

    if not cap.isOpened():
        raise RuntimeError(f"Failed to open video: {args.video}")

    source_fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    video_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    video_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    if source_fps <= 0 or not np.isfinite(source_fps):
        source_fps = 30.0

    output_width = video_width * 2
    output_height = video_height

    args.output.parent.mkdir(parents=True, exist_ok=True)

    fourcc = cv2.VideoWriter_fourcc(*args.codec)
    writer = cv2.VideoWriter(
        str(args.output),
        fourcc,
        source_fps,
        (output_width, output_height),
    )

    if not writer.isOpened():
        raise RuntimeError(
            f"Failed to open VideoWriter: {args.output}. "
            f"Try --codec avc1 or --codec XVID."
        )

    print(f"TensorRT version: {trt.__version__}")
    print(f"GPU: {torch.cuda.get_device_name(device)}")
    print(f"CUDA_VISIBLE_DEVICES: {os.environ.get('CUDA_VISIBLE_DEVICES')}")
    print(f"Visible CUDA device: {device}")
    print(f"Engine load: {time.perf_counter() - load_start:.3f} s")

    print(f"Video: {args.video}")
    print(f"Video info: {video_width}x{video_height}, source_fps={source_fps:.2f}, frames={total_frames}")
    print(f"Output: {args.output}")
    print(f"Output info: {output_width}x{output_height}, fps={source_fps:.2f}, codec={args.codec}")
    print(f"Preprocess: RGB={not args.bgr}, imagenet_normalization={not args.no_normalize}")
    print(f"Depth output tensor: {depth_name}")

    for name in input_names:
        print(f"Input : {name} shape={tuple(tensors[name].shape)} dtype={tensors[name].dtype}")

    for name in output_names:
        print(f"Output: {name} shape={tuple(tensors[name].shape)} dtype={tensors[name].dtype}")

    normalize = not args.no_normalize
    rgb = not args.bgr

    # Warm-up using first frame, then rewind.
    ret, first_frame = cap.read()
    if not ret:
        raise RuntimeError(f"Failed to read first frame from video: {args.video}")

    with torch.cuda.stream(stream), torch.inference_mode():
        print("\nRunning warm-up...")

        for _ in range(args.warmup):
            preprocess_frame(
                frame_bgr=first_frame,
                input_tensor=tensors[input_name],
                device=device,
                normalize=normalize,
                rgb=rgb,
            )

            ok = context.execute_async_v3(stream.cuda_stream)

            if not ok:
                raise RuntimeError("TensorRT warm-up execution failed")

        stream.synchronize()

    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    frames_to_process = total_frames
    if args.max_frames > 0:
        frames_to_process = min(total_frames, args.max_frames)

    print("\nRunning video inference and recording...")

    gpu_times_ms = []
    pipeline_times_ms = []
    write_times_ms = []

    processed = 0
    wall_start = time.perf_counter()

    with torch.cuda.stream(stream), torch.inference_mode():
        while processed < frames_to_process:
            ret, frame = cap.read()

            if not ret:
                break

            iter_start = time.perf_counter()

            preprocess_frame(
                frame_bgr=frame,
                input_tensor=tensors[input_name],
                device=device,
                normalize=normalize,
                rgb=rgb,
            )

            start_event = torch.cuda.Event(enable_timing=True)
            end_event = torch.cuda.Event(enable_timing=True)

            start_event.record(stream)

            ok = context.execute_async_v3(stream.cuda_stream)

            if not ok:
                raise RuntimeError("TensorRT execution failed")

            end_event.record(stream)
            end_event.synchronize()

            gpu_ms = start_event.elapsed_time(end_event)
            gpu_times_ms.append(gpu_ms)

            # Copy depth output to CPU after inference is complete.
            depth_np = tensors[depth_name].detach().cpu().numpy()

            depth_color = colorize_depth(
                depth=depth_np,
                out_w=video_width,
                out_h=video_height,
                invert=args.invert_depth,
            )

            current_fps = 1000.0 / max(1e-6, gpu_ms)
            side_by_side = make_side_by_side(
                frame_bgr=frame,
                depth_color=depth_color,
                frame_index=processed,
                fps_text=f"TRT: {current_fps:.1f} FPS",
            )

            write_start = time.perf_counter()
            writer.write(side_by_side)
            write_times_ms.append((time.perf_counter() - write_start) * 1000.0)

            pipeline_times_ms.append((time.perf_counter() - iter_start) * 1000.0)

            processed += 1

            if processed % 100 == 0:
                elapsed = time.perf_counter() - wall_start
                print(
                    f"Processed {processed}/{frames_to_process} frames, "
                    f"avg throughput={processed / elapsed:.2f} FPS"
                )

    wall_elapsed = time.perf_counter() - wall_start

    cap.release()
    writer.release()

    if processed == 0:
        raise RuntimeError("No frames were processed")

    gpu_mean_ms = statistics.fmean(gpu_times_ms)
    pipeline_mean_ms = statistics.fmean(pipeline_times_ms)
    write_mean_ms = statistics.fmean(write_times_ms) if write_times_ms else 0.0

    pure_engine_fps = 1000.0 / gpu_mean_ms
    pipeline_fps_by_mean = 1000.0 / pipeline_mean_ms
    wall_loop_fps = processed / wall_elapsed

    allocated_mb = torch.cuda.memory_allocated(device) / 1024**2
    reserved_mb = torch.cuda.memory_reserved(device) / 1024**2

    print("\n=== DA3 TensorRT Video Recording Result ===")
    print(f"Input video             : {args.video}")
    print(f"Output video            : {args.output}")
    print(f"Processed frames        : {processed}")
    print(f"Source FPS              : {source_fps:.2f}")

    print("\n--- TensorRT Engine Only ---")
    print(f"GPU latency mean        : {gpu_mean_ms:.3f} ms")
    print(f"GPU latency median      : {statistics.median(gpu_times_ms):.3f} ms")
    print(f"GPU latency p95         : {percentile(gpu_times_ms, 95):.3f} ms")
    print(f"GPU latency p99         : {percentile(gpu_times_ms, 99):.3f} ms")
    print(f"Pure-engine FPS         : {pure_engine_fps:.2f}")

    print("\n--- Full Recording Pipeline ---")
    print("Included: video read, preprocess, TensorRT inference, depth CPU copy, colorize, side-by-side concat, video write")
    print(f"Pipeline latency mean   : {pipeline_mean_ms:.3f} ms")
    print(f"Pipeline latency median : {statistics.median(pipeline_times_ms):.3f} ms")
    print(f"Pipeline latency p95    : {percentile(pipeline_times_ms, 95):.3f} ms")
    print(f"Pipeline latency p99    : {percentile(pipeline_times_ms, 99):.3f} ms")
    print(f"Pipeline FPS by mean    : {pipeline_fps_by_mean:.2f}")
    print(f"Wall-loop throughput    : {wall_loop_fps:.2f} FPS")
    print(f"Video write mean        : {write_mean_ms:.3f} ms")

    print("\n--- Memory ---")
    print(f"Torch allocated/reserved: {allocated_mb:.1f}/{reserved_mb:.1f} MiB")

    print("\nDone.")


if __name__ == "__main__":
    main()
