#!/usr/bin/env python3
"""Safely collect Corina V2 parallel-ankle motor/IMU calibration data.

The tool is deliberately read-only unless ``--execute`` and the explicit
fixture confirmation are both supplied.  It supports the FDILink binary IMU
stream observed on the Corina test machine:

* packet 0x40: calibrated gyro/accelerometer/magnetometer data
* packet 0x41: AHRS Euler angles and quaternion (w, x, y, z)

Motor commands are raw A/B encoder targets around the positions measured when
the program starts.  The tested pair has its URDF limits removed in the local
runtime config because those limits describe virtual ankle Pitch/Roll, not the
parallel mechanism's physical motor coordinates.  The tool supplies its own
small relative motor travel and IMU attitude limits instead.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass, replace
from datetime import datetime
import json
import math
import os
from pathlib import Path
import select
import struct
import termios
import threading
import time
from typing import Iterable, Sequence

from arx_d_can import ArxDCanArm, default_config


EXECUTE_CONFIRMATION = "FIXED_AND_SUSPENDED"
FDILINK_START = 0xFC
FDILINK_END = 0xFD
FDILINK_MSG_IMU = 0x40
FDILINK_MSG_AHRS = 0x41


class CalibrationSafetyError(RuntimeError):
    """Raised when a runtime safety invariant is violated."""


@dataclass(frozen=True, slots=True)
class FdilinkFrame:
    packet_id: int
    sequence: int
    payload: bytes
    host_monotonic_ns: int


@dataclass(frozen=True, slots=True)
class ImuData:
    sequence: int
    host_monotonic_ns: int
    gyro_x: float
    gyro_y: float
    gyro_z: float
    accel_x: float
    accel_y: float
    accel_z: float
    mag_x: float
    mag_y: float
    mag_z: float
    temperature_c: float
    pressure_pa: float
    pressure_temperature_c: float
    device_timestamp_us: int


@dataclass(frozen=True, slots=True)
class AhrsData:
    sequence: int
    host_monotonic_ns: int
    roll_speed: float
    pitch_speed: float
    heading_speed: float
    roll: float
    pitch: float
    heading: float
    qw: float
    qx: float
    qy: float
    qz: float
    device_timestamp_us: int

    @property
    def quaternion(self) -> tuple[float, float, float, float]:
        return normalize_quaternion((self.qw, self.qx, self.qy, self.qz))


@dataclass(frozen=True, slots=True)
class ImuSnapshot:
    imu: ImuData
    ahrs: AhrsData
    host_monotonic_ns: int


@dataclass(frozen=True, slots=True)
class ScanPoint:
    label: str
    a_relative_deg: float
    b_relative_deg: float


def crc8_dallas(data: bytes) -> int:
    """FDILink header CRC8 (Dallas/Maxim reflected polynomial 0x8C)."""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8C) if crc & 1 else (crc >> 1)
    return crc & 0xFF


def crc16_xmodem(data: bytes) -> int:
    """FDILink payload CRC16-XMODEM (polynomial 0x1021, initial 0)."""
    crc = 0
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


class FdilinkParser:
    """Incremental, resynchronizing FDILink frame parser."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.bad_frames = 0

    def feed(self, data: bytes, *, host_monotonic_ns: int | None = None) -> list[FdilinkFrame]:
        if data:
            self._buffer.extend(data)
        received_ns = time.monotonic_ns() if host_monotonic_ns is None else host_monotonic_ns
        frames: list[FdilinkFrame] = []
        while True:
            try:
                start = self._buffer.index(FDILINK_START)
            except ValueError:
                self._buffer.clear()
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 5:
                break
            payload_length = self._buffer[2]
            frame_length = payload_length + 8
            if len(self._buffer) < frame_length:
                break
            candidate = bytes(self._buffer[:frame_length])
            header_ok = crc8_dallas(candidate[:4]) == candidate[4]
            payload = candidate[7:-1]
            expected_crc = (candidate[5] << 8) | candidate[6]
            payload_ok = crc16_xmodem(payload) == expected_crc
            footer_ok = candidate[-1] == FDILINK_END
            if not (header_ok and payload_ok and footer_ok):
                self.bad_frames += 1
                del self._buffer[0]
                continue
            frames.append(
                FdilinkFrame(
                    packet_id=candidate[1],
                    sequence=candidate[3],
                    payload=payload,
                    host_monotonic_ns=received_ns,
                )
            )
            del self._buffer[:frame_length]
        return frames


def decode_imu_frame(frame: FdilinkFrame) -> ImuData:
    if frame.packet_id != FDILINK_MSG_IMU or len(frame.payload) != 56:
        raise ValueError("expected FDILink 0x40 IMU payload with length 56")
    values = struct.unpack("<12fq", frame.payload)
    return ImuData(frame.sequence, frame.host_monotonic_ns, *values)


def decode_ahrs_frame(frame: FdilinkFrame) -> AhrsData:
    if frame.packet_id != FDILINK_MSG_AHRS or len(frame.payload) != 48:
        raise ValueError("expected FDILink 0x41 AHRS payload with length 48")
    values = struct.unpack("<10fq", frame.payload)
    result = AhrsData(frame.sequence, frame.host_monotonic_ns, *values)
    normalize_quaternion(result.quaternion)
    return result


def normalize_quaternion(values: Sequence[float]) -> tuple[float, float, float, float]:
    if len(values) != 4 or any(not math.isfinite(float(value)) for value in values):
        raise ValueError("quaternion must contain four finite values")
    norm = math.sqrt(sum(float(value) ** 2 for value in values))
    if norm < 1e-9:
        raise ValueError("quaternion norm is zero")
    return tuple(float(value) / norm for value in values)  # type: ignore[return-value]


def quaternion_conjugate(q: Sequence[float]) -> tuple[float, float, float, float]:
    w, x, y, z = normalize_quaternion(q)
    return w, -x, -y, -z


def quaternion_multiply(
    left: Sequence[float], right: Sequence[float]
) -> tuple[float, float, float, float]:
    w1, x1, y1, z1 = normalize_quaternion(left)
    w2, x2, y2, z2 = normalize_quaternion(right)
    return normalize_quaternion(
        (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )
    )


def quaternion_to_rpy(q: Sequence[float]) -> tuple[float, float, float]:
    """Return intrinsic XYZ roll/pitch/yaw in radians."""
    w, x, y, z = normalize_quaternion(q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_sine = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_sine)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def average_quaternions(quaternions: Sequence[Sequence[float]]) -> tuple[float, float, float, float]:
    if not quaternions:
        raise ValueError("at least one quaternion is required")
    reference = normalize_quaternion(quaternions[0])
    accumulator = [0.0, 0.0, 0.0, 0.0]
    for values in quaternions:
        current = normalize_quaternion(values)
        if sum(a * b for a, b in zip(reference, current)) < 0.0:
            current = tuple(-value for value in current)
        for index, value in enumerate(current):
            accumulator[index] += value
    return normalize_quaternion(accumulator)


def _baud_constant(baud: int) -> int:
    name = f"B{baud}"
    if not hasattr(termios, name):
        raise ValueError(f"unsupported POSIX serial baud rate: {baud}")
    return int(getattr(termios, name))


def configure_raw_serial(fd: int, baud: int) -> None:
    speed = _baud_constant(baud)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
    attrs[2] &= ~getattr(termios, "CRTSCTS", 0)
    attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = speed
    attrs[5] = speed
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)


class FdilinkImuReader:
    """Background read-only FDILink serial reader."""

    def __init__(self, port: str, baud: int = 921600) -> None:
        self.port = port
        self.baud = baud
        self._fd: int | None = None
        self._parser = FdilinkParser()
        self._imu: ImuData | None = None
        self._ahrs: AhrsData | None = None
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: BaseException | None = None

    @property
    def bad_frames(self) -> int:
        return self._parser.bad_frames

    def start(self) -> None:
        if self._thread is not None:
            return
        self._fd = os.open(self.port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        configure_raw_serial(self._fd, self.baud)
        self._thread = threading.Thread(target=self._run, name="fdilink-imu", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._fd is not None:
            os.close(self._fd)
        self._fd = None
        self._thread = None

    def _run(self) -> None:
        assert self._fd is not None
        try:
            while not self._stop.is_set():
                readable, _, _ = select.select([self._fd], [], [], 0.1)
                if not readable:
                    continue
                chunk = os.read(self._fd, 4096)
                if not chunk:
                    continue
                for frame in self._parser.feed(chunk):
                    with self._condition:
                        if frame.packet_id == FDILINK_MSG_IMU:
                            self._imu = decode_imu_frame(frame)
                        elif frame.packet_id == FDILINK_MSG_AHRS:
                            self._ahrs = decode_ahrs_frame(frame)
                        else:
                            continue
                        self._condition.notify_all()
        except BaseException as exc:  # propagate serial/parser failures to control thread
            with self._condition:
                self._error = exc
                self._condition.notify_all()

    def snapshot(self, *, timeout: float = 1.0, max_age_s: float = 0.2) -> ImuSnapshot:
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._imu is None or self._ahrs is None:
                if self._error is not None:
                    raise RuntimeError(f"IMU reader failed: {self._error}") from self._error
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError("timed out waiting for FDILink IMU and AHRS packets")
                self._condition.wait(remaining)
            if self._error is not None:
                raise RuntimeError(f"IMU reader failed: {self._error}") from self._error
            now_ns = time.monotonic_ns()
            oldest_sensor_ns = min(self._imu.host_monotonic_ns, self._ahrs.host_monotonic_ns)
            age_s = (now_ns - oldest_sensor_ns) / 1e9
            if age_s > max_age_s:
                raise TimeoutError(f"IMU data is stale ({age_s:.3f}s > {max_age_s:.3f}s)")
            return ImuSnapshot(self._imu, self._ahrs, now_ns)


def parse_positive_floats(text: str) -> list[float]:
    values = [float(part.strip()) for part in text.split(",") if part.strip()]
    if not values or any(not math.isfinite(value) or value <= 0.0 for value in values):
        raise argparse.ArgumentTypeError("expected comma-separated positive finite values")
    return values


def inclusive_range(start: float, stop: float, step: float) -> list[float]:
    if not all(math.isfinite(value) for value in (start, stop, step)) or step <= 0.0:
        raise ValueError("grid bounds must be finite and step must be positive")
    if start > stop:
        raise ValueError("grid minimum must not exceed maximum")
    count = int(math.floor((stop - start) / step + 1e-9))
    values = [start + index * step for index in range(count + 1)]
    if not math.isclose(values[-1], stop, abs_tol=1e-9):
        values.append(stop)
    return values


def micro_scan_points(deltas_deg: Sequence[float]) -> list[ScanPoint]:
    points = [ScanPoint("zero", 0.0, 0.0)]
    for delta in deltas_deg:
        tests = (
            ("a_pos", delta, 0.0),
            ("a_neg", -delta, 0.0),
            ("b_pos", 0.0, delta),
            ("b_neg", 0.0, -delta),
            ("common_pos", delta, delta),
            ("common_neg", -delta, -delta),
            ("differential_pos", delta, -delta),
            ("differential_neg", -delta, delta),
        )
        for label, a_value, b_value in tests:
            points.append(ScanPoint(f"{label}_{delta:g}deg", a_value, b_value))
            points.append(ScanPoint("return_zero", 0.0, 0.0))
    return points


def grid_scan_points(
    *, a_min: float, a_max: float, a_step: float, b_min: float, b_max: float, b_step: float
) -> list[ScanPoint]:
    a_values = inclusive_range(a_min, a_max, a_step)
    b_values = inclusive_range(b_min, b_max, b_step)
    points = [ScanPoint("zero", 0.0, 0.0)]
    for row, b_value in enumerate(b_values):
        row_a = a_values if row % 2 == 0 else list(reversed(a_values))
        for a_value in row_a:
            points.append(ScanPoint(f"grid_r{row}", a_value, b_value))
    points.append(ScanPoint("return_zero", 0.0, 0.0))
    return points


def ankle_names(side: str) -> tuple[str, str]:
    return f"{side}_leg_joint5", f"{side}_leg_joint6"


def build_runtime_config(args: argparse.Namespace):
    config = default_config(
        model="corina_v2",
        port=args.port,
        baud=args.baud,
        control_hz=args.hz,
        arm_control_mode="mit",
    )
    test_names = set(ankle_names(args.side))
    side_prefix = f"{args.side}_leg_"
    # These two URDF limits are virtual Pitch/Roll limits.  Raw A/B commands
    # are instead constrained relative to the measured startup encoders below.
    joints = tuple(
        replace(joint, lower_limit=None, upper_limit=None)
        if joint.name in test_names
        else joint
        for joint in config.arm_joints
        if joint.name.startswith(side_prefix)
    )
    return replace(config, arm_joints=joints)


def relative_orientation(
    zero_quaternion: Sequence[float], current_quaternion: Sequence[float]
) -> tuple[tuple[float, float, float, float], tuple[float, float, float]]:
    relative = quaternion_multiply(quaternion_conjugate(zero_quaternion), current_quaternion)
    return relative, quaternion_to_rpy(relative)


def vector_norm(values: Iterable[float]) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in values))


def mean(values: Sequence[float]) -> float:
    return sum(values) / len(values)


def standard_deviation(values: Sequence[float]) -> float:
    if len(values) < 2:
        return 0.0
    average = mean(values)
    return math.sqrt(sum((value - average) ** 2 for value in values) / (len(values) - 1))


class CalibrationRecorder:
    def __init__(self, output: Path, *, joint_names: Sequence[str], metadata: dict) -> None:
        output.parent.mkdir(parents=True, exist_ok=True)
        self.output = output
        self.summary_path = output.with_name(f"{output.stem}_points.csv")
        self.metadata_path = output.with_name(f"{output.stem}_metadata.json")
        self._raw_file = output.open("w", newline="", encoding="utf-8")
        self._summary_file = self.summary_path.open("w", newline="", encoding="utf-8")
        self.joint_names = tuple(joint_names)
        self.raw_fields = [
            "host_unix_ns", "host_monotonic_ns", "side", "phase", "point_index", "point_label",
            "settled", "a_command_relative_deg", "b_command_relative_deg",
            "a_command_absolute_deg", "b_command_absolute_deg",
            "a_actual_relative_deg", "b_actual_relative_deg",
            "a_actual_absolute_deg", "b_actual_absolute_deg",
            "a_velocity_deg_s", "b_velocity_deg_s", "a_torque_nm", "b_torque_nm",
            "imu_sequence", "ahrs_sequence", "imu_device_timestamp_us", "ahrs_device_timestamp_us",
            "imu_qw", "imu_qx", "imu_qy", "imu_qz",
            "relative_qw", "relative_qx", "relative_qy", "relative_qz",
            "roll_relative_deg", "pitch_relative_deg", "yaw_relative_deg",
            "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s",
            "accel_x_m_s2", "accel_y_m_s2", "accel_z_m_s2", "imu_temperature_c",
            "joint_positions_deg", "joint_velocities_deg_s", "joint_torques_nm",
        ]
        self.summary_fields = [
            "side", "point_index", "point_label", "sample_count",
            "a_command_relative_deg", "b_command_relative_deg",
            "a_actual_relative_mean_deg", "a_actual_relative_std_deg",
            "b_actual_relative_mean_deg", "b_actual_relative_std_deg",
            "roll_relative_mean_deg", "roll_relative_std_deg",
            "pitch_relative_mean_deg", "pitch_relative_std_deg",
            "a_torque_abs_max_nm", "b_torque_abs_max_nm",
        ]
        self._raw_writer = csv.DictWriter(self._raw_file, fieldnames=self.raw_fields)
        self._summary_writer = csv.DictWriter(self._summary_file, fieldnames=self.summary_fields)
        self._raw_writer.writeheader()
        self._summary_writer.writeheader()
        self.metadata_path.write_text(json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8")

    def close(self) -> None:
        self._raw_file.close()
        self._summary_file.close()

    def write_raw(self, row: dict) -> None:
        self._raw_writer.writerow({field: row.get(field, "") for field in self.raw_fields})
        self._raw_file.flush()

    def write_summary(self, row: dict) -> None:
        self._summary_writer.writerow({field: row.get(field, "") for field in self.summary_fields})
        self._summary_file.flush()


class AnkleCalibrationRunner:
    def __init__(
        self,
        args: argparse.Namespace,
        arm: ArxDCanArm,
        imu: FdilinkImuReader,
        recorder: CalibrationRecorder,
        initial_positions: Sequence[float],
        zero_quaternion: Sequence[float],
    ) -> None:
        self.args = args
        self.arm = arm
        self.imu = imu
        self.recorder = recorder
        self.initial = tuple(float(value) for value in initial_positions)
        self.command = list(self.initial)
        self.zero_quaternion = normalize_quaternion(zero_quaternion)
        a_name, b_name = ankle_names(args.side)
        self.a_index = arm.joint_names.index(a_name)
        self.b_index = arm.joint_names.index(b_name)
        self.test_indices = {self.a_index, self.b_index}
        self.hold_indices = set(range(len(self.initial))) - self.test_indices
        self.kp = [joint.mit_kp for joint in arm.config.arm_joints]
        self.kd = [joint.mit_kd for joint in arm.config.arm_joints]
        self.kp[self.a_index] = args.test_kp
        self.kp[self.b_index] = args.test_kp
        self.kd[self.a_index] = args.test_kd
        self.kd[self.b_index] = args.test_kd
        self.zeros = [0.0] * len(self.initial)

    def _send(self, target: Sequence[float], *, require_enabled: bool = True) -> None:
        self.arm.send_joint_positions(
            target,
            velocities=self.zeros,
            torques=self.zeros,
            mit_kp=self.kp,
            mit_kd=self.kd,
            mode="mit",
            require_enabled=require_enabled,
        )

    def _sample(
        self,
        *,
        phase: str,
        point_index: int,
        point: ScanPoint,
        settled: bool,
        enforce_tracking: bool,
    ) -> dict:
        state = self.arm.read_state(request_feedback=True).arm
        snapshot = self.imu.snapshot(max_age_s=self.args.max_imu_age)
        current_q = snapshot.ahrs.quaternion
        relative_q, (roll, pitch, yaw) = relative_orientation(self.zero_quaternion, current_q)
        positions = tuple(float(value) for value in state.positions)
        velocities = tuple(float(value) for value in state.velocities)
        torques = tuple(float(value) for value in state.torques)
        self._check_safety(
            positions=positions,
            velocities=velocities,
            torques=torques,
            roll=roll,
            pitch=pitch,
            enforce_tracking=enforce_tracking,
        )
        degrees = 180.0 / math.pi
        row = {
            "host_unix_ns": time.time_ns(),
            "host_monotonic_ns": snapshot.host_monotonic_ns,
            "side": self.args.side,
            "phase": phase,
            "point_index": point_index,
            "point_label": point.label,
            "settled": int(settled),
            "a_command_relative_deg": math.degrees(self.command[self.a_index] - self.initial[self.a_index]),
            "b_command_relative_deg": math.degrees(self.command[self.b_index] - self.initial[self.b_index]),
            "a_command_absolute_deg": math.degrees(self.command[self.a_index]),
            "b_command_absolute_deg": math.degrees(self.command[self.b_index]),
            "a_actual_relative_deg": math.degrees(positions[self.a_index] - self.initial[self.a_index]),
            "b_actual_relative_deg": math.degrees(positions[self.b_index] - self.initial[self.b_index]),
            "a_actual_absolute_deg": math.degrees(positions[self.a_index]),
            "b_actual_absolute_deg": math.degrees(positions[self.b_index]),
            "a_velocity_deg_s": math.degrees(velocities[self.a_index]),
            "b_velocity_deg_s": math.degrees(velocities[self.b_index]),
            "a_torque_nm": torques[self.a_index],
            "b_torque_nm": torques[self.b_index],
            "imu_sequence": snapshot.imu.sequence,
            "ahrs_sequence": snapshot.ahrs.sequence,
            "imu_device_timestamp_us": snapshot.imu.device_timestamp_us,
            "ahrs_device_timestamp_us": snapshot.ahrs.device_timestamp_us,
            "imu_qw": current_q[0], "imu_qx": current_q[1], "imu_qy": current_q[2], "imu_qz": current_q[3],
            "relative_qw": relative_q[0], "relative_qx": relative_q[1],
            "relative_qy": relative_q[2], "relative_qz": relative_q[3],
            "roll_relative_deg": roll * degrees,
            "pitch_relative_deg": pitch * degrees,
            "yaw_relative_deg": yaw * degrees,
            "gyro_x_rad_s": snapshot.imu.gyro_x,
            "gyro_y_rad_s": snapshot.imu.gyro_y,
            "gyro_z_rad_s": snapshot.imu.gyro_z,
            "accel_x_m_s2": snapshot.imu.accel_x,
            "accel_y_m_s2": snapshot.imu.accel_y,
            "accel_z_m_s2": snapshot.imu.accel_z,
            "imu_temperature_c": snapshot.imu.temperature_c,
            "joint_positions_deg": json.dumps([value * degrees for value in positions]),
            "joint_velocities_deg_s": json.dumps([value * degrees for value in velocities]),
            "joint_torques_nm": json.dumps(torques),
        }
        self.recorder.write_raw(row)
        return row

    def _check_safety(
        self,
        *,
        positions: Sequence[float],
        velocities: Sequence[float],
        torques: Sequence[float],
        roll: float,
        pitch: float,
        enforce_tracking: bool,
    ) -> None:
        for index in self.test_indices:
            actual_delta = abs(math.degrees(positions[index] - self.initial[index]))
            if actual_delta > self.args.max_motor_delta:
                raise CalibrationSafetyError(
                    f"{self.arm.joint_names[index]} moved {actual_delta:.2f}deg from startup "
                    f"(limit {self.args.max_motor_delta:.2f}deg)"
                )
            if abs(torques[index]) > self.args.max_test_torque:
                raise CalibrationSafetyError(
                    f"{self.arm.joint_names[index]} torque {torques[index]:+.3f}Nm exceeds "
                    f"{self.args.max_test_torque:.3f}Nm"
                )
            if enforce_tracking:
                error = abs(math.degrees(self.command[index] - positions[index]))
                if error > self.args.max_tracking_error:
                    raise CalibrationSafetyError(
                        f"{self.arm.joint_names[index]} tracking error {error:.2f}deg exceeds "
                        f"{self.args.max_tracking_error:.2f}deg"
                    )
        for index in self.hold_indices:
            drift = abs(math.degrees(positions[index] - self.initial[index]))
            if drift > self.args.max_hold_drift:
                raise CalibrationSafetyError(
                    f"held joint {self.arm.joint_names[index]} drifted {drift:.2f}deg"
                )
        roll_deg = abs(math.degrees(roll))
        pitch_deg = abs(math.degrees(pitch))
        if roll_deg > self.args.max_roll:
            raise CalibrationSafetyError(
                f"relative IMU roll {roll_deg:.2f}deg exceeds {self.args.max_roll:.2f}deg"
            )
        if pitch_deg > self.args.max_pitch:
            raise CalibrationSafetyError(
                f"relative IMU pitch {pitch_deg:.2f}deg exceeds {self.args.max_pitch:.2f}deg"
            )

    def _target_for_point(self, point: ScanPoint) -> list[float]:
        if max(abs(point.a_relative_deg), abs(point.b_relative_deg)) > self.args.max_motor_delta:
            raise ValueError(
                f"point {point.label} exceeds --max-motor-delta {self.args.max_motor_delta:g}deg"
            )
        target = list(self.initial)
        target[self.a_index] += math.radians(point.a_relative_deg)
        target[self.b_index] += math.radians(point.b_relative_deg)
        return target

    def move_to(self, point: ScanPoint, *, point_index: int) -> None:
        target = self._target_for_point(point)
        start = list(self.command)
        intervals = max(1, int(math.ceil(self.args.move_seconds * self.args.hz)))
        started = time.perf_counter()
        for step in range(1, intervals + 1):
            t = step / intervals
            scale = 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5
            self.command = [a + (b - a) * scale for a, b in zip(start, target)]
            self._send(self.command)
            self._sample(
                phase="move", point_index=point_index, point=point,
                settled=False, enforce_tracking=False,
            )
            deadline = started + step * self.args.move_seconds / intervals
            remaining = deadline - time.perf_counter()
            if remaining > 0.0:
                time.sleep(remaining)
        self.command = target

    def settle_and_record(self, point: ScanPoint, *, point_index: int) -> None:
        stable_since: float | None = None
        deadline = time.monotonic() + self.args.settle_timeout
        while time.monotonic() < deadline:
            self._send(self.command)
            row = self._sample(
                phase="settle", point_index=point_index, point=point,
                settled=False, enforce_tracking=False,
            )
            motor_speed = max(abs(row["a_velocity_deg_s"]), abs(row["b_velocity_deg_s"]))
            gyro_speed = math.degrees(
                vector_norm((row["gyro_x_rad_s"], row["gyro_y_rad_s"], row["gyro_z_rad_s"]))
            )
            stable = motor_speed <= self.args.max_settle_motor_speed and gyro_speed <= self.args.max_settle_gyro
            now = time.monotonic()
            stable_since = now if stable and stable_since is None else stable_since
            if not stable:
                stable_since = None
            if stable_since is not None and now - stable_since >= self.args.stable_seconds:
                break
            time.sleep(1.0 / self.args.hz)
        else:
            raise CalibrationSafetyError(f"point {point.label} did not settle before timeout")

        samples: list[dict] = []
        sample_deadline = time.monotonic() + self.args.sample_seconds
        while time.monotonic() < sample_deadline:
            self._send(self.command)
            samples.append(
                self._sample(
                    phase="sample", point_index=point_index, point=point,
                    settled=True, enforce_tracking=True,
                )
            )
            time.sleep(1.0 / self.args.hz)
        if not samples:
            raise CalibrationSafetyError("no settled samples were recorded")
        self.recorder.write_summary(
            {
                "side": self.args.side,
                "point_index": point_index,
                "point_label": point.label,
                "sample_count": len(samples),
                "a_command_relative_deg": point.a_relative_deg,
                "b_command_relative_deg": point.b_relative_deg,
                "a_actual_relative_mean_deg": mean([row["a_actual_relative_deg"] for row in samples]),
                "a_actual_relative_std_deg": standard_deviation([row["a_actual_relative_deg"] for row in samples]),
                "b_actual_relative_mean_deg": mean([row["b_actual_relative_deg"] for row in samples]),
                "b_actual_relative_std_deg": standard_deviation([row["b_actual_relative_deg"] for row in samples]),
                "roll_relative_mean_deg": mean([row["roll_relative_deg"] for row in samples]),
                "roll_relative_std_deg": standard_deviation([row["roll_relative_deg"] for row in samples]),
                "pitch_relative_mean_deg": mean([row["pitch_relative_deg"] for row in samples]),
                "pitch_relative_std_deg": standard_deviation([row["pitch_relative_deg"] for row in samples]),
                "a_torque_abs_max_nm": max(abs(row["a_torque_nm"]) for row in samples),
                "b_torque_abs_max_nm": max(abs(row["b_torque_nm"]) for row in samples),
            }
        )
        print(
            f"[{point_index:03d}] {point.label:<24} "
            f"cmd(A,B)=({point.a_relative_deg:+.2f},{point.b_relative_deg:+.2f})deg "
            f"actual=({mean([r['a_actual_relative_deg'] for r in samples]):+.2f},"
            f"{mean([r['b_actual_relative_deg'] for r in samples]):+.2f})deg "
            f"IMU(P,R)=({mean([r['pitch_relative_deg'] for r in samples]):+.2f},"
            f"{mean([r['roll_relative_deg'] for r in samples]):+.2f})deg",
            flush=True,
        )

    def run(self, points: Sequence[ScanPoint]) -> None:
        # Preload current encoder targets while motors are still disabled, then
        # enable and immediately refresh the same command.
        self._send(self.command, require_enabled=False)
        self.arm.enable()
        self._send(self.command)
        for point_index, point in enumerate(points):
            self.move_to(point, point_index=point_index)
            self.settle_and_record(point, point_index=point_index)
        if not (
            math.isclose(self.command[self.a_index], self.initial[self.a_index], abs_tol=1e-9)
            and math.isclose(self.command[self.b_index], self.initial[self.b_index], abs_tol=1e-9)
        ):
            final_point = ScanPoint("final_return_zero", 0.0, 0.0)
            self.move_to(final_point, point_index=len(points))
            self.settle_and_record(final_point, point_index=len(points))


def collect_imu_zero(
    reader: FdilinkImuReader,
    *,
    seconds: float,
    max_age_s: float,
    max_gyro_deg_s: float,
) -> tuple[float, float, float, float]:
    deadline = time.monotonic() + seconds
    quaternions: list[tuple[float, float, float, float]] = []
    gyro_norms_deg_s: list[float] = []
    seen: tuple[int, int] | None = None
    while time.monotonic() < deadline:
        snapshot = reader.snapshot(timeout=1.0, max_age_s=max_age_s)
        key = (snapshot.ahrs.sequence, snapshot.ahrs.device_timestamp_us)
        if key != seen:
            quaternions.append(snapshot.ahrs.quaternion)
            gyro_norms_deg_s.append(
                math.degrees(
                    vector_norm(
                        (snapshot.imu.gyro_x, snapshot.imu.gyro_y, snapshot.imu.gyro_z)
                    )
                )
            )
            seen = key
        time.sleep(0.005)
    if len(quaternions) < 10:
        raise RuntimeError(f"only received {len(quaternions)} AHRS samples during zero capture")
    average_gyro = mean(gyro_norms_deg_s)
    if average_gyro > max_gyro_deg_s:
        raise CalibrationSafetyError(
            f"foot IMU is not stationary during zero capture: average gyro "
            f"{average_gyro:.3f}deg/s > {max_gyro_deg_s:.3f}deg/s"
        )
    return average_quaternions(quaternions)


def default_output_path(side: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("calibration_data") / f"corina_{side}_ankle_{stamp}.csv"


def build_points(args: argparse.Namespace) -> list[ScanPoint]:
    if args.plan == "micro":
        return micro_scan_points(args.deltas)
    return grid_scan_points(
        a_min=args.a_min, a_max=args.a_max, a_step=args.a_step,
        b_min=args.b_min, b_max=args.b_max, b_step=args.b_step,
    )


def print_initial_state(joint_names: Sequence[str], positions: Sequence[float]) -> None:
    print("motor feedback:")
    for name, position in zip(joint_names, positions):
        print(f"  {name:<20} {math.degrees(position):+9.3f} deg")


def run(args: argparse.Namespace) -> int:
    if args.execute and args.confirm != EXECUTE_CONFIRMATION:
        raise SystemExit(
            f"--execute requires --confirm {EXECUTE_CONFIRMATION}; "
            "mechanically support the robot and fix the tested shank first"
        )
    if args.test_kp <= 0.0 or args.test_kd < 0.0:
        raise SystemExit("--test-kp must be positive and --test-kd must be non-negative")
    positive_parameters = {
        "--hz": args.hz,
        "--move-seconds": args.move_seconds,
        "--settle-timeout": args.settle_timeout,
        "--stable-seconds": args.stable_seconds,
        "--sample-seconds": args.sample_seconds,
        "--zero-seconds": args.zero_seconds,
        "--max-motor-delta": args.max_motor_delta,
        "--max-test-torque": args.max_test_torque,
        "--max-tracking-error": args.max_tracking_error,
        "--max-hold-drift": args.max_hold_drift,
        "--max-pitch": args.max_pitch,
        "--max-roll": args.max_roll,
        "--max-settle-motor-speed": args.max_settle_motor_speed,
        "--max-initial-motor-speed": args.max_initial_motor_speed,
        "--max-settle-gyro": args.max_settle_gyro,
        "--max-imu-age": args.max_imu_age,
    }
    invalid = [name for name, value in positive_parameters.items() if not math.isfinite(value) or value <= 0.0]
    if invalid:
        raise SystemExit("these parameters must be positive and finite: " + ", ".join(invalid))
    output = args.output or default_output_path(args.side)
    config = build_runtime_config(args)
    arm = ArxDCanArm(config=config)
    imu = FdilinkImuReader(args.imu_port, args.imu_baud)
    recorder: CalibrationRecorder | None = None
    try:
        imu.start()
        first_imu = imu.snapshot(timeout=3.0, max_age_s=args.max_imu_age)
        print(
            f"IMU ready: port={args.imu_port} baud={args.imu_baud} "
            f"q=({first_imu.ahrs.qw:+.4f},{first_imu.ahrs.qx:+.4f},"
            f"{first_imu.ahrs.qy:+.4f},{first_imu.ahrs.qz:+.4f})"
        )
        arm.connect()
        initial_state = arm.read_state(request_feedback=True).arm
        print_initial_state(initial_state.names, initial_state.positions)
        status_codes = arm.robot.get_status_codes(joint_names=list(arm.joint_names))
        enabled_names = [name for name, status in status_codes.items() if status == 1]
        fault_names = [name for name, status in status_codes.items() if status not in (0, 1)]
        print(
            "motor status: enabled="
            + (", ".join(enabled_names) if enabled_names else "none")
            + " faults="
            + (", ".join(fault_names) if fault_names else "none")
        )
        if args.execute and (enabled_names or fault_names):
            raise CalibrationSafetyError(
                "all motors must report DISABLED and fault-free before automatic calibration"
            )
        initial_max_speed = max(abs(math.degrees(value)) for value in initial_state.velocities)
        if args.execute and initial_max_speed > args.max_initial_motor_speed:
            raise CalibrationSafetyError(
                f"robot is not stationary: maximum motor speed {initial_max_speed:.3f}deg/s "
                f"> {args.max_initial_motor_speed:.3f}deg/s"
            )
        print(f"capturing stationary IMU zero for {args.zero_seconds:g}s ...")
        zero_quaternion = collect_imu_zero(
            imu,
            seconds=args.zero_seconds,
            max_age_s=args.max_imu_age,
            max_gyro_deg_s=args.max_settle_gyro,
        )
        print("IMU zero quaternion:", " ".join(f"{value:+.7f}" for value in zero_quaternion))
        if not args.execute:
            print(
                "READ-ONLY VALIDATION PASSED: motors were not configured or enabled.\n"
                f"To run the {args.plan} plan, add: --execute --confirm {EXECUTE_CONFIRMATION}"
            )
            return 0

        points = build_points(args)
        metadata = {
            "created_at": datetime.now().isoformat(),
            "model": "corina_v2",
            "motor_port": args.port,
            "imu_port": args.imu_port,
            "imu_baud": args.imu_baud,
            "side": args.side,
            "plan": args.plan,
            "joint_names": list(arm.joint_names),
            "initial_positions_rad": list(initial_state.positions),
            "initial_positions_deg": [math.degrees(value) for value in initial_state.positions],
            "imu_zero_quaternion_wxyz": list(zero_quaternion),
            "arguments": {
                key: str(value) if isinstance(value, Path) else value
                for key, value in vars(args).items()
            },
            "points": [asdict(point) for point in points],
            "notes": (
                "IMU axes must be aligned with the foot axes. The tested motor pair uses runtime "
                "relative travel limits; URDF joint5/joint6 limits are virtual ankle limits."
            ),
        }
        recorder = CalibrationRecorder(output, joint_names=arm.joint_names, metadata=metadata)
        arm.configure("mit")
        runner = AnkleCalibrationRunner(
            args, arm, imu, recorder, initial_state.positions, zero_quaternion
        )
        print(f"executing {len(points)} points; raw log: {output}")
        runner.run(points)
        print(f"completed: {output}")
        print(f"point summary: {recorder.summary_path}")
        print(f"metadata: {recorder.metadata_path}")
        return 0
    except KeyboardInterrupt:
        print("\ninterrupted: disabling all motors immediately", flush=True)
        return 130
    except CalibrationSafetyError as exc:
        print(f"SAFETY STOP: {exc}", flush=True)
        return 2
    finally:
        if arm.connected:
            try:
                if arm.enabled:
                    arm.disable()
            finally:
                # disable() above performs the verified physical stop.  The
                # close call only releases the transport, which also keeps the
                # default validation path strictly read-only.
                arm.close(disable=False)
        imu.close()
        if recorder is not None:
            recorder.close()


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description=(
            "Collect synchronized Corina V2 raw ankle A/B encoder and FDILink foot-IMU data. "
            "Default mode is read-only validation."
        )
    )
    result.add_argument("--side", choices=("right", "left"), default="right")
    result.add_argument("--port", default="/dev/ttyACM0", help="Corina USB2CAN serial port")
    result.add_argument("--baud", type=int, default=1_000_000)
    result.add_argument("--imu-port", default="/dev/ttyUSB0")
    result.add_argument("--imu-baud", type=int, default=921600)
    result.add_argument("--output", type=Path, default=None)
    result.add_argument("--execute", action="store_true", help="Enable motors and execute the scan")
    result.add_argument("--confirm", default="", help=f"Required with --execute: {EXECUTE_CONFIRMATION}")
    result.add_argument("--plan", choices=("micro", "grid"), default="micro")
    result.add_argument("--deltas", type=parse_positive_floats, default=[1.0], help="Micro-test deltas in degrees")
    result.add_argument("--a-min", type=float, default=-3.0)
    result.add_argument("--a-max", type=float, default=3.0)
    result.add_argument("--a-step", type=float, default=1.0)
    result.add_argument("--b-min", type=float, default=-3.0)
    result.add_argument("--b-max", type=float, default=3.0)
    result.add_argument("--b-step", type=float, default=1.0)
    result.add_argument("--hz", type=float, default=50.0)
    result.add_argument("--move-seconds", type=float, default=2.0)
    result.add_argument("--settle-timeout", type=float, default=5.0)
    result.add_argument("--stable-seconds", type=float, default=0.5)
    result.add_argument("--sample-seconds", type=float, default=1.0)
    result.add_argument("--zero-seconds", type=float, default=2.0)
    result.add_argument("--test-kp", type=float, default=2.0)
    result.add_argument("--test-kd", type=float, default=0.5)
    result.add_argument("--max-motor-delta", type=float, default=5.0, help="Raw A/B travel from startup, degrees")
    result.add_argument("--max-test-torque", type=float, default=3.0, help="Absolute A/B feedback torque, Nm")
    result.add_argument("--max-tracking-error", type=float, default=3.0, help="Settled A/B error, degrees")
    result.add_argument("--max-hold-drift", type=float, default=2.0, help="Other-joint drift, degrees")
    result.add_argument("--max-pitch", type=float, default=28.0, help="Relative IMU pitch soft limit, degrees")
    result.add_argument("--max-roll", type=float, default=12.0, help="Relative IMU roll soft limit, degrees")
    result.add_argument("--max-settle-motor-speed", type=float, default=0.8, help="A/B settle speed, deg/s")
    result.add_argument(
        "--max-initial-motor-speed",
        type=float,
        default=2.0,
        help="Maximum motor speed allowed before enabling, deg/s",
    )
    result.add_argument("--max-settle-gyro", type=float, default=1.5, help="IMU settle gyro norm, deg/s")
    result.add_argument("--max-imu-age", type=float, default=0.2, help="Maximum IMU packet age, seconds")
    return result


def main() -> None:
    raise SystemExit(run(parser().parse_args()))


if __name__ == "__main__":
    main()
