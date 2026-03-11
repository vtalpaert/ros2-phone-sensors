#!/usr/bin/env python3
"""
Evaluate IMU noise and quantization from two rosbag recordings.

Two recordings are required:
  1. Static bag  (hours, phone completely still):
       - estimates noise density for gyro and accel
       - estimates orientation variance
       - detects orientation quantization step (static is fine: signal is small but
         still visits a few discrete levels over hours)

  2. Dynamic bag (1-2 min, phone moved in all directions):
       - detects gyro and accel quantization step
       - NOT used for noise estimation (motion contaminates std)
       - NOT used for orientation quantization (signal changes too fast)

Usage:
    python3 imu_gaussian_noise.py <static_bag> <dynamic_bag> [options]

    # Skip vibration disturbances in the static bag:
    python3 imu_gaussian_noise.py static/ dynamic/ --start 60 --end 3000
"""

import argparse
import sys
import numpy as np
from scipy.spatial.transform import Rotation

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_imu_from_bag(bag_path: str, topic: str = "/imu", start_sec: float = None, end_sec: float = None):
    """Read orientation (quaternion), gyro, and accel arrays from an MCAP bag."""
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    if topic not in type_map:
        available = [t.name for t in topic_types]
        print(f"ERROR: topic '{topic}' not found. Available: {available}", file=sys.stderr)
        sys.exit(1)

    msg_type = get_message(type_map[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    timestamps, orientation, gyro, accel = [], [], [], []
    t0 = None

    while reader.has_next():
        (_, data, stamp_ns) = reader.read_next()
        t_sec = stamp_ns * 1e-9
        if t0 is None:
            t0 = t_sec
        t_rel = t_sec - t0

        if start_sec is not None and t_rel < start_sec:
            continue
        if end_sec is not None and t_rel > end_sec:
            break

        msg = deserialize_message(data, msg_type)
        timestamps.append(t_sec)
        orientation.append([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        gyro.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        accel.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    return np.array(timestamps), np.array(orientation), np.array(gyro), np.array(accel)


def estimate_sampling_frequency(timestamps: np.ndarray) -> float:
    dt = np.diff(timestamps)
    return 1.0 / np.median(dt[dt > 0])


def detect_quantization_step(data_1d: np.ndarray) -> tuple:
    """
    Find the quantization step of a 1D signal, robust to outliers and flat signals.

    Strategy:
      - Remove outliers (lone peaks) via percentile clipping at 0.5/99.5 first.
      - Detect flat signals on the clipped inliers via ptp (peak-to-peak). IQR is
        wrong here: a signal that spends >50% of time at one discrete level has IQR=0
        even though it is not flat.
      - Round inliers to 7 significant figures of the data range before np.unique.
        This collapses floating-point noise (~1e-15) introduced by coordinate transforms
        (e.g. quaternion → Euler) without merging adjacent quantization levels (~1e-3).
        For gyro/accel the values are exact floats from CDR, so rounding has no effect.

    Returns (step, is_flat): step is None when signal is flat or undetectable.
    """
    # Remove outliers before any analysis
    p_low, p_high = np.percentile(data_1d, [0.5, 99.5])
    inliers = data_1d[(data_1d >= p_low) & (data_1d <= p_high)]
    if len(inliers) < 10:
        return None, False

    # Flat detection after outlier removal
    if np.ptp(inliers) < 1e-15:
        return None, True

    # Round to 7 significant figures of the data range to collapse FP noise
    precision = np.ptp(inliers) * 1e-7
    rounded = np.round(inliers / precision) * precision

    unique_vals = np.unique(rounded)
    if len(unique_vals) < 2:
        return None, True

    gaps = np.diff(unique_vals)
    positive_gaps = gaps[gaps > 0]
    if len(positive_gaps) == 0:
        return None, True

    step = np.min(positive_gaps)
    return step, False


def rate_noise_analysis(data: np.ndarray, fs: float, axis_names: list, label: str):
    """
    Estimate noise density for gyro or accel from a STATIC recording.
        noise_density = std / sqrt(fs)  [unit/sqrt(Hz)]
        discrete_var  = fs * noise_density²  [unit²]  — Gaussian contribution per sample
    """
    print(f"\n{'='*60}")
    print(f"  {label}")
    print(f"{'='*60}")
    print(f"  Samples: {len(data)}  |  Duration: {len(data)/fs:.1f} s  |  fs = {fs:.2f} Hz")
    print()

    densities = []
    for i, ax in enumerate(axis_names):
        mean = np.mean(data[:, i])
        std = np.std(data[:, i], ddof=1)
        density = std / np.sqrt(fs)
        discrete_var = fs * density ** 2  # == std², sanity check
        densities.append(density)
        print(f"  {ax}:  mean = {mean:+.6f}   std = {std:.6f}   noise_density = {density:.6e}   discrete_var = {discrete_var:.6e}")

    mean_density = np.mean(densities)
    print()
    print(f"  Mean noise density (all axes): {mean_density:.6e}")
    print(f"  Mean discrete_var  (all axes): {fs * mean_density**2:.6e}  (= fs × sigma²)")
    return mean_density


def orientation_noise_analysis(quaternions: np.ndarray, fs: float):
    """
    Estimate orientation covariance and quantization step from a STATIC recording.

    Covariance is std² [rad²] directly — no fs scaling — because orientation is
    an absolute angle estimate from the phone's sensor fusion, not a rate measurement.

    Quantization is detectable from the static bag: even 2-3 discrete levels over
    hours of data are enough to identify the grid step.
    """
    print(f"\n{'='*60}")
    print("  ORIENTATION  [rad]")
    print(f"{'='*60}")
    print(f"  Samples: {len(quaternions)}  |  Duration: {len(quaternions)/fs:.1f} s  |  fs = {fs:.2f} Hz")
    print()

    rpy = np.unwrap(Rotation.from_quat(quaternions).as_euler("xyz", degrees=False), axis=0)

    variances = []
    steps = []
    for i, ax in enumerate(["roll (x)", "pitch (y)", "yaw (z)"]):
        mean = np.mean(rpy[:, i])
        std = np.std(rpy[:, i], ddof=1)
        var = std ** 2
        variances.append(var)

        step, is_flat = detect_quantization_step(rpy[:, i])
        if step is not None and not is_flat:
            steps.append(step)
            var_q = step ** 2 / 12
            step_str = f"{step:.6f} rad  ({np.degrees(step):.4f}°)   var_q = {var_q:.6e} rad²"
        elif is_flat:
            step_str = "flat"
        else:
            step_str = "undetectable"

        print(f"  {ax}:")
        print(f"    mean = {np.degrees(mean):+.4f}°   std = {np.degrees(std):.4f}° ({std:.6f} rad)   variance = {var:.6e} rad²")
        print(f"    quantization step: {step_str}")

    mean_var = np.mean(variances)
    mean_step = np.mean(steps) if steps else None
    print()
    print(f"  Mean variance (all axes): {mean_var:.6e} rad²")
    print(f"  Note: this is std², not a noise density — no fs scaling applies.")
    return mean_var, mean_step


def quantization_analysis(data: np.ndarray, axis_names: list, label: str):
    """
    Detect quantization step from a DYNAMIC recording (phone moved in all directions).
    The wide signal range covers many discrete levels, making the step unambiguous.
    Flat axes (e.g. an accelerometer axis that always outputs 0) are flagged explicitly.
    Returns mean_step across detectable axes, or None.
    """
    print(f"\n{'='*60}")
    print(f"  {label}")
    print(f"{'='*60}")
    print()

    steps = []
    for i, ax in enumerate(axis_names):
        step, is_flat = detect_quantization_step(data[:, i])
        if is_flat:
            print(f"  {ax}: signal flat — axis appears inactive or gravity-cancelled")
        elif step is not None:
            var_q = step ** 2 / 12
            steps.append(step)
            print(f"  {ax}: quantization step = {step:.6e}   var_q = {var_q:.6e}")
        else:
            print(f"  {ax}: quantization step undetectable")

    return np.mean(steps) if steps else None


def print_ros_snippet(
    gyro_density: float, accel_density: float, orientation_var: float,
    fs: float, gyro_step, accel_step, orientation_step
):
    var_g_gaussian = fs * gyro_density ** 2
    var_a_gaussian = fs * accel_density ** 2
    var_g_quant = gyro_step ** 2 / 12 if gyro_step is not None else float("nan")
    var_a_quant = accel_step ** 2 / 12 if accel_step is not None else float("nan")
    var_ori_quant = orientation_step ** 2 / 12 if orientation_step is not None else float("nan")

    print(f"\n{'='*60}")
    print("  VARIANCE COMPARISON  (before inflation, at measured fs)")
    print(f"{'='*60}")
    print(f"  {'':20s}  {'Gaussian (fs×σ²)':>18s}  {'Quantization (q²/12)':>20s}  {'Sum':>12s}")
    print(f"  {'-'*76}")
    print(f"  {'gyro   [rad/s]²':20s}  {var_g_gaussian:18.6e}  {var_g_quant:20.6e}  {var_g_gaussian + var_g_quant:12.6e}")
    print(f"  {'accel  [m/s²]²':20s}  {var_a_gaussian:18.6e}  {var_a_quant:20.6e}  {var_a_gaussian + var_a_quant:12.6e}")
    print(f"  {'orient [rad²]':20s}  {orientation_var:18.6e}  {var_ori_quant:20.6e}  {orientation_var + var_ori_quant:12.6e}")
    print()
    print("  Both sources are independent → variances sum (do not take max).")
    print()

    k = 10
    var_g_total_at_fs  = k * var_g_gaussian + var_g_quant
    var_a_total_at_fs  = k * var_a_gaussian + var_a_quant
    var_ori_total      = orientation_var + var_ori_quant

    print(f"{'='*60}")
    print("  Suggested values for message_converters.py")
    print(f"{'='*60}")
    print()
    print("  GYROSCOPE  [rad/s]")
    print(f"    noise_density  = {gyro_density:.4e}  # rad/s/sqrt(Hz)  → dynamic part (computed from fs)")
    print(f"    var_quant      = {var_g_quant:.4e}  # rad²            → fixed part (q²/12)")
    print(f"    k_inflation    = {k}                       # suggested safety factor")
    print(f"    total at fs    = k × fs × sigma² + var_quant  =  {var_g_total_at_fs:.4e}  rad²")
    print()
    print("  ACCELEROMETER  [m/s²]")
    print(f"    noise_density  = {accel_density:.4e}  # m/s²/sqrt(Hz)  → dynamic part (computed from fs)")
    print(f"    var_quant      = {var_a_quant:.4e}  # (m/s²)²        → fixed part (q²/12)")
    print(f"    k_inflation    = {k}                       # suggested safety factor")
    print(f"    total at fs    = k × fs × sigma² + var_quant  =  {var_a_total_at_fs:.4e}  (m/s²)²")
    print()
    print("  ORIENTATION  [rad]")
    print(f"    var_measured   = {orientation_var:.4e}  # rad²  (std² from static recording)")
    print(f"    var_quant      = {var_ori_quant:.4e}  # rad²  (q²/12)")
    print(f"    total (fixed)  = var_measured + var_quant  =  {var_ori_total:.4e}  rad²")
    print()
    print("  NOTE: gyro/accel have a fixed (var_quant) + dynamic (k × fs × sigma²) part.")
    print("        orientation uses a single fixed variance (no fs scaling).")


def main():
    parser = argparse.ArgumentParser(
        description="Estimate IMU Gaussian noise and quantization from two MCAP rosbags.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "static_bag : long recording with the phone completely still\n"
            "dynamic_bag: short recording (1-2 min) with the phone moved in all directions"
        ),
    )
    parser.add_argument("static_bag", help="Long static recording")
    parser.add_argument("dynamic_bag", help="Short dynamic recording (all orientations)")
    parser.add_argument("--topic", default="/imu", help="IMU topic name (default: /imu)")
    parser.add_argument("--start", type=float, default=None, metavar="SEC",
                        help="Start offset into static bag (seconds) to skip disturbances")
    parser.add_argument("--end", type=float, default=None, metavar="SEC",
                        help="End offset into static bag (seconds)")
    args = parser.parse_args()

    print(f"Static bag : {args.static_bag}")
    if args.start or args.end:
        print(f"  Window: [{args.start or 0:.1f}s, {args.end or 'end'}s]")
    print(f"Dynamic bag: {args.dynamic_bag}")

    print("\nReading static bag...")
    ts_static, ori_static, gyro_static, accel_static = read_imu_from_bag(
        args.static_bag, args.topic, args.start, args.end
    )
    print("Reading dynamic bag...")
    _, _, gyro_dynamic, accel_dynamic = read_imu_from_bag(args.dynamic_bag, args.topic)

    if len(ts_static) == 0:
        print("ERROR: No messages found in static bag window.", file=sys.stderr)
        sys.exit(1)
    if len(gyro_dynamic) == 0:
        print("ERROR: No messages found in dynamic bag.", file=sys.stderr)
        sys.exit(1)

    fs = estimate_sampling_frequency(ts_static)

    print("\n" + "#" * 60)
    print("  NOISE ESTIMATION  (static bag)")
    print("#" * 60)
    orientation_var, orientation_step = orientation_noise_analysis(ori_static, fs)
    gyro_density = rate_noise_analysis(gyro_static, fs, ["gx", "gy", "gz"], "GYROSCOPE  [rad/s]")
    accel_density = rate_noise_analysis(accel_static, fs, ["ax", "ay", "az"], "ACCELEROMETER  [m/s2]")

    print("\n" + "#" * 60)
    print("  QUANTIZATION DETECTION  (dynamic bag)")
    print("#" * 60)
    gyro_step = quantization_analysis(gyro_dynamic, ["gx", "gy", "gz"], "GYROSCOPE  [rad/s]")
    accel_step = quantization_analysis(accel_dynamic, ["ax", "ay", "az"], "ACCELEROMETER  [m/s2]")

    print_ros_snippet(gyro_density, accel_density, orientation_var, fs, gyro_step, accel_step, orientation_step)
    print()


if __name__ == "__main__":
    main()
