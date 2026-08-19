"""Replay a recorded Yunyi VR trajectory through the public raw MIT API.

This is an opt-in real-hardware diagnostic.  It deliberately performs no
per-frame terminal or disk I/O.  A passive SocketCAN listener retains only the
last few seconds in memory and writes them after the run has stopped.
"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict, is_dataclass, replace
import json
import os
from pathlib import Path
import select
import socket
import struct
import sys
import threading
import time
from typing import Any

from arx_d_can import ArxDCanDualArm


DEFAULT_TRACE = Path(
    os.environ.get(
        "ARX_D_CAN_VR_TRACE",
        "/home/ubuntu/vr-pico/logs/yunyi_v1_0_sessions/"
        "20260817_184647_pid2041205_control_100hz.jsonl",
    )
)
VR_SOURCE = Path(
    os.environ.get("ARX_D_CAN_VR_SOURCE", "/home/ubuntu/vr-pico/src")
)
CAN_RAW_FD_FRAMES = 5
CAN_RAW_FILTER = 1
CANFD_BRS = 0x01
CANFD_FRAME = struct.Struct("=IBBBB64s")
RIGHT_MOTOR_LIMITS = {
    1: (45.0, 54.0),
    2: (45.0, 54.0),
    3: (10.0, 28.0),
    4: (10.0, 28.0),
    5: (30.0, 10.0),
    6: (30.0, 10.0),
    7: (30.0, 10.0),
    8: (30.0, 10.0),
}


def _jsonable(value: Any) -> Any:
    if is_dataclass(value):
        return {key: _jsonable(item) for key, item in asdict(value).items()}
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if hasattr(value, "name") and hasattr(value, "value"):
        return value.name
    return value


class CanMemoryTrace:
    """Passive CAN-FD capture with a time-bounded in-memory ring."""

    def __init__(self, interfaces: tuple[str, ...], history_seconds: float) -> None:
        self.interfaces = interfaces
        self.history_ns = int(history_seconds * 1e9)
        # 20k frames/s leaves ample headroom above two 8-motor 500 Hz buses.
        self.events: deque[dict[str, Any]] = deque(
            maxlen=max(4096, int(history_seconds * 20_000))
        )
        # Linux does not expose the primary controller socket's receive queue
        # depth to this passive socket. Keep this explicit instead of
        # presenting the passive listener's private queue as controller data.
        self.queue_high_water_bytes = {name: None for name in interfaces}
        self.capture_errors: list[str] = []
        self._sockets: dict[socket.socket, str] = {}
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        for interface in self.interfaces:
            sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
            sock.setblocking(False)
            sock.bind((interface,))
            self._sockets[sock] = interface
        self._thread = threading.Thread(
            target=self._run,
            name="yunyi-can-memory-trace",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        for sock in self._sockets:
            sock.close()
        self._sockets.clear()

    def _run(self) -> None:
        sockets = tuple(self._sockets)
        while not self._stop.is_set():
            try:
                readable, _, _ = select.select(sockets, (), (), 0.02)
                for sock in readable:
                    interface = self._sockets[sock]
                    while True:
                        try:
                            raw = sock.recv(CANFD_FRAME.size)
                        except BlockingIOError:
                            break
                        received_ns = time.monotonic_ns()
                        if len(raw) == CANFD_FRAME.size:
                            can_id, length, flags, _, _, data = CANFD_FRAME.unpack(raw)
                        elif len(raw) == 16:
                            can_id, length, data = struct.unpack("=IB3x8s", raw)
                            flags = 0
                        else:
                            self.capture_errors.append(
                                f"{interface}: unexpected SocketCAN frame size {len(raw)}"
                            )
                            continue
                        arbitration_id = can_id & 0x1FFFFFFF
                        register_reply = (
                            length == 8
                            and data[1] <= 0x0F
                            and data[2] in (0x33, 0x55, 0xAA)
                        )
                        self.events.append(
                            {
                                "monotonic_ns": received_ns,
                                "interface": interface,
                                "can_id": arbitration_id,
                                "flags": flags,
                                "brs": bool(flags & CANFD_BRS),
                                "length": length,
                                "data": data[:length].hex(),
                                "kind": (
                                    "register"
                                    if register_reply
                                    else
                                    "command"
                                    if 1 <= arbitration_id <= 8
                                    else "feedback"
                                    if (
                                        0x11 <= arbitration_id <= 0x18
                                        or 0x201 <= arbitration_id <= 0x208
                                    )
                                    else "other"
                                ),
                            }
                        )
                cutoff = time.monotonic_ns() - self.history_ns
                while self.events and self.events[0]["monotonic_ns"] < cutoff:
                    self.events.popleft()
            except Exception as exc:  # diagnostic failure must not stop control
                self.capture_errors.append(f"{type(exc).__name__}: {exc}")
                time.sleep(0.01)

    def summary(self) -> dict[str, Any]:
        events = list(self.events)
        feedback_intervals: dict[str, list[int]] = {}
        previous: dict[str, int] = {}
        counts: dict[str, int] = {}
        for event in events:
            key = f"{event['interface']}:0x{event['can_id']:X}:{event['kind']}"
            counts[key] = counts.get(key, 0) + 1
            if event["kind"] != "feedback":
                continue
            if key in previous:
                feedback_intervals.setdefault(key, []).append(
                    event["monotonic_ns"] - previous[key]
                )
            previous[key] = event["monotonic_ns"]
        return {
            "retained_events": len(events),
            "counts": counts,
            "max_feedback_interval_ms": {
                key: max(values) / 1e6
                for key, values in feedback_intervals.items()
                if values
            },
            "queue_high_water_bytes": self.queue_high_water_bytes,
            "capture_errors": self.capture_errors,
            "right_load": self._right_load_summary(events),
        }

    @staticmethod
    def _right_load_summary(events) -> dict[str, Any]:
        """Decode feedback load without claiming it is 24 V input current."""
        peaks = {
            motor_id: {
                "abs_velocity_rad_s": 0.0,
                "abs_torque_nm": 0.0,
                "abs_mechanical_power_w": 0.0,
                "max_mos_c": 0,
                "max_rotor_c": 0,
                "fault_statuses": set(),
            }
            for motor_id in RIGHT_MOTOR_LIMITS
        }
        latest = {}
        peak_motoring_w = 0.0
        peak_regenerative_w = 0.0
        for event in events:
            if event["interface"] != "can-right" or event["kind"] != "feedback":
                continue
            motor_id = event["can_id"] & 0x0F
            if motor_id not in RIGHT_MOTOR_LIMITS:
                continue
            data = bytes.fromhex(event["data"])
            if len(data) != 8:
                continue
            velocity_limit, torque_limit = RIGHT_MOTOR_LIMITS[motor_id]
            velocity_raw = (data[3] << 4) | (data[4] >> 4)
            torque_raw = ((data[4] & 0x0F) << 8) | data[5]
            velocity = velocity_raw * (2.0 * velocity_limit / 4095.0) - velocity_limit
            torque = torque_raw * (2.0 * torque_limit / 4095.0) - torque_limit
            power = velocity * torque
            value = peaks[motor_id]
            value["abs_velocity_rad_s"] = max(value["abs_velocity_rad_s"], abs(velocity))
            value["abs_torque_nm"] = max(value["abs_torque_nm"], abs(torque))
            value["abs_mechanical_power_w"] = max(
                value["abs_mechanical_power_w"], abs(power)
            )
            value["max_mos_c"] = max(value["max_mos_c"], data[6])
            value["max_rotor_c"] = max(value["max_rotor_c"], data[7])
            status = data[0] >> 4
            if status > 1:
                value["fault_statuses"].add(status)
            latest[motor_id] = (event["monotonic_ns"], power)
            if motor_id == 8 and len(latest) == 8:
                newest = event["monotonic_ns"]
                if all(newest - item[0] <= 5_000_000 for item in latest.values()):
                    motoring = sum(max(0.0, item[1]) for item in latest.values())
                    regenerative = sum(max(0.0, -item[1]) for item in latest.values())
                    peak_motoring_w = max(peak_motoring_w, motoring)
                    peak_regenerative_w = max(peak_regenerative_w, regenerative)
        serializable = {}
        for motor_id, value in peaks.items():
            serializable[str(motor_id)] = {
                key: sorted(item) if isinstance(item, set) else item
                for key, item in value.items()
            }
        return {
            "per_motor_peaks": serializable,
            "peak_positive_mechanical_power_w": peak_motoring_w,
            "peak_regenerative_mechanical_power_w": peak_regenerative_w,
            "ideal_24v_current_lower_bound_a": peak_motoring_w / 24.0,
            "note": (
                "mechanical power/24V is only an ideal lower bound; holding, copper, "
                "inverter and transient input currents require electrical measurement"
            ),
        }

    def mailbox_summary(self, submissions) -> dict[str, Any]:
        """Correlate caller submissions with observed worker TX generations.

        ABI 2.5 does not expose the mailbox generation counter.  A can0 ID-1
        MIT frame is the first arm frame of each dual worker transaction, so
        it is a low-disturbance wire-level proxy for consumption.  Results are
        labelled estimates rather than pretending to be native counters.
        """
        tx = [
            event["monotonic_ns"]
            for event in self.events
            if event["interface"] == "can-left"
            and event["can_id"] == 1
            and event["kind"] == "command"
            and event["data"] not in {
                "fffffffffffffffc",
                "fffffffffffffffd",
                "fffffffffffffffb",
            }
        ]
        submitted = [item["monotonic_ns"] for item in submissions]
        if not tx or not submitted:
            return {
                "method": "wire_correlation_estimate",
                "observed_submissions": len(submitted),
                "observed_worker_cycles": len(tx),
                "estimated_overwrites": None,
                "consume_latency_ms": None,
            }
        start = tx[0]
        submission_index = 0
        previous_tx = start - 10_000_000
        overwrites = 0
        latencies = []
        for sent_ns in tx:
            window = []
            while submission_index < len(submitted) and submitted[submission_index] <= sent_ns:
                value = submitted[submission_index]
                if value > previous_tx:
                    window.append(value)
                submission_index += 1
            if window:
                overwrites += max(0, len(window) - 1)
                latencies.append(sent_ns - window[-1])
            previous_tx = sent_ns
        return {
            "method": "wire_correlation_estimate",
            "observed_submissions": len(submitted),
            "observed_worker_cycles": len(tx),
            "estimated_overwrites": overwrites,
            "consume_latency_ms": None
            if not latencies
            else {
                "min": min(latencies) / 1e6,
                "mean": sum(latencies) / len(latencies) / 1e6,
                "max": max(latencies) / 1e6,
            },
        }

    def dump(self, path: Path, metadata: dict[str, Any], submissions=()) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as stream:
            stream.write(json.dumps({"record_type": "summary", **metadata}) + "\n")
            for submission in submissions:
                stream.write(
                    json.dumps({"record_type": "raw_submission", **submission}) + "\n"
                )
            for event in self.events:
                stream.write(json.dumps({"record_type": "can", **event}) + "\n")


class RegisterSampler:
    """Low-rate VBus reads on an independent filtered SocketCAN socket."""

    def __init__(
        self,
        interface: str,
        motors: dict[int, tuple[str, int]],
        sample_hz: float,
    ) -> None:
        self.interface = interface
        self.motors = motors
        self.sample_hz = sample_hz
        self.samples: list[dict[str, Any]] = []
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self.sample_hz <= 0:
            return
        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
        filters = b"".join(
            struct.pack("=II", feedback_id, 0x7FF)
            for _, feedback_id in self.motors.values()
        )
        sock.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_FILTER, filters)
        sock.setblocking(False)
        sock.bind((self.interface,))
        self._socket = sock
        self._thread = threading.Thread(
            target=self._run,
            name="yunyi-vbus-register-sampler",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        sock = getattr(self, "_socket", None)
        if sock is not None:
            sock.close()
            self._socket = None

    def _run(self) -> None:
        period = 1.0 / self.sample_hz
        deadline = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if now >= deadline:
                for motor_id in self.motors:
                    payload = bytes((motor_id & 0xFF, motor_id >> 8, 0x33, 60, 0, 0, 0, 0))
                    frame = CANFD_FRAME.pack(
                        0x7FF,
                        8,
                        CANFD_BRS,
                        0,
                        0,
                        payload.ljust(64, b"\0"),
                    )
                    try:
                        self._socket.send(frame)
                    except Exception as exc:
                        self.samples.append(
                            {
                                "monotonic_ns": time.monotonic_ns(),
                                "motor": self.motors[motor_id][0],
                                "error": f"send {type(exc).__name__}: {exc}",
                            }
                        )
                deadline += period
                if deadline < now:
                    deadline = now + period
            readable, _, _ = select.select(
                (self._socket,),
                (),
                (),
                min(0.01, max(0.0, deadline - time.monotonic())),
            )
            if not readable:
                continue
            while True:
                try:
                    raw = self._socket.recv(CANFD_FRAME.size)
                except BlockingIOError:
                    break
                received_ns = time.monotonic_ns()
                if len(raw) != CANFD_FRAME.size:
                    continue
                _, length, _, _, _, data = CANFD_FRAME.unpack(raw)
                if not (
                    length == 8
                    and data[1] <= 0x0F
                    and data[2] == 0x33
                    and data[3] == 60
                ):
                    continue
                motor_id = data[0] | (data[1] << 8)
                if motor_id not in self.motors:
                    continue
                sample = {
                    "monotonic_ns": received_ns,
                    "motor": self.motors[motor_id][0],
                    "vbus_v": struct.unpack("<f", data[4:8])[0],
                }
                self.samples.append(sample)

    def summary(self) -> dict[str, Any]:
        per_motor: dict[str, dict[str, Any]] = {}
        for name, _ in self.motors.values():
            values = [
                item["vbus_v"]
                for item in self.samples
                if item["motor"] == name and "vbus_v" in item
            ]
            errors = sum(
                1
                for item in self.samples
                if item["motor"] == name and "error" in item
            )
            per_motor[name] = {
                "sample_count": len(values),
                "error_count": errors,
                "min_v": None if not values else min(values),
                "max_v": None if not values else max(values),
            }
        return {
            "requested_sweep_hz": self.sample_hz,
            "register": 60,
            "per_motor": per_motor,
            "samples": self.samples,
            "note": "read-only diagnostic traffic; not a substitute for oscilloscope capture",
        }


def _load_frames(trace_path: Path, control_hz: int):
    if control_hz not in {400, 500}:
        raise ValueError("control_hz must be 400 or 500 for this 100 Hz trace")
    sys.path.insert(0, str(VR_SOURCE))
    from teleop.adapters.arm.raw_mit_interpolation import (  # noqa: PLC0415
        DualArmRawMitInterpolator,
    )

    targets: list[tuple[list[float], list[float]]] = []
    with trace_path.open("r", encoding="utf-8") as stream:
        for line in stream:
            value = json.loads(line)
            if value.get("record_type") != "control_tick":
                continue
            targets.append(
                (
                    value["left"]["command_target_rad"],
                    value["right"]["command_target_rad"],
                )
            )
    interpolator = DualArmRawMitInterpolator(
        "hermite",
        substeps=control_hz // 100,
        source_period_s=0.01,
    )
    frames = []
    for left, right in targets:
        frames.extend(interpolator.add_target(left, right))
    frames.extend(interpolator.flush())
    expected = (len(targets) - 1) * (control_hz // 100)
    if len(frames) != expected:
        raise RuntimeError(f"generated {len(frames)} frames, expected {expected}")
    return targets, frames


def _motor_stats(robot: ArxDCanDualArm) -> dict[str, Any]:
    output: dict[str, Any] = {}
    for arm in (robot._left, robot._right):
        for name, motor in arm.robot._motor_map.items():
            state = motor.get_state()
            output[name] = {
                "feedback": _jsonable(motor.get_feedback_stats()),
                "integrity": _jsonable(motor.get_feedback_integrity_stats()),
                "state": None if state is None else _jsonable(state),
            }
    return output


def _wait_at_start(
    robot: ArxDCanDualArm,
    left,
    right,
    timeout_s: float,
    velocity: float,
) -> None:
    robot.set_joint_mit(left=left, right=right, velocity=velocity)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = robot.read_cached_state()
        error = max(
            *(abs(a - b) for a, b in zip(state.left.arm.positions, left)),
            *(abs(a - b) for a, b in zip(state.right.arm.positions, right)),
        )
        if error < 0.05:
            return
        health = robot.safety_health
        if health.fault_reason:
            raise RuntimeError(f"fault while moving to replay start: {health.fault_reason}")
        time.sleep(0.1)
    raise RuntimeError("robot did not reach the VR replay start pose")


def run(args: argparse.Namespace) -> int:
    targets, frames = _load_frames(args.trace, args.control_hz)
    capture = CanMemoryTrace(("can-left", "can-right"), args.history_seconds)
    robot = ArxDCanDualArm(control_mode="mit")
    # The product profile defaults to 500 Hz.  A/B replay must change both
    # inputs to the Runtime constructor before connect without editing the
    # installed product YAML between runs.
    robot._left.config = replace(robot._left.config, control_hz=float(args.control_hz))
    robot._right.config = replace(robot._right.config, control_hz=float(args.control_hz))
    for arm in (robot._left, robot._right):
        arm.config = replace(
            arm.config,
            max_cached_feedback_age_s=args.feedback_max_age_ms / 1000.0,
            feedback_fault_threshold=args.feedback_failure_threshold,
            feedback_check_hz=float(args.feedback_check_hz),
        )
    result: dict[str, Any] = {
        "trace": str(args.trace),
        "source_targets": len(targets),
        "generated_frames": len(frames),
        "requested_control_hz": args.control_hz,
        "j12_only": args.j12_only,
        "motion_scale": args.motion_scale,
        "j12_kp_scale": args.j12_kp_scale,
        "j12_kd_scale": args.j12_kd_scale,
        "feedback_max_age_ms": args.feedback_max_age_ms,
        "feedback_failure_threshold": args.feedback_failure_threshold,
        "feedback_check_hz": args.feedback_check_hz,
        "preposition_only": args.preposition_only,
        "started_wall_time_ns": time.time_ns(),
    }
    failure: BaseException | None = None
    submitted_frames = 0
    raw_started: float | None = None
    vbus_sampler: RegisterSampler | None = None
    submission_events: deque[dict[str, int]] = deque(
        maxlen=max(2048, int(args.history_seconds * args.control_hz * 2))
    )
    capture.start()
    try:
        robot.connect()
        if robot._effective_control_hz != args.control_hz:
            raise RuntimeError(
                f"Runtime control_hz={robot._effective_control_hz}, expected {args.control_hz}"
            )
        robot.enable()
        vbus_sampler = RegisterSampler(
            "can-right",
            {
                1: ("r-joint1", 0x201),
                2: ("r-joint2", 0x202),
            },
            args.vbus_sample_hz,
        )
        vbus_sampler.start()
        _wait_at_start(
            robot,
            targets[0][0],
            targets[0][1],
            args.start_timeout_s,
            args.start_velocity,
        )
        before = _motor_stats(robot)
        period = 1.0 / args.control_hz
        started = time.perf_counter()
        raw_started = started
        run_frames = () if args.preposition_only else frames
        right_origin = tuple(float(value) for value in targets[0][1])
        kp = None
        if args.j12_kp_scale != 1.0:
            kp = [joint.mit_kp for joint in robot._right.config.arm_joints]
            kp[0] *= args.j12_kp_scale
            kp[1] *= args.j12_kp_scale
        kd = None
        if args.j12_kd_scale != 1.0:
            kd = [joint.mit_kd for joint in robot._right.config.arm_joints]
            kd[0] *= args.j12_kd_scale
            kd[1] *= args.j12_kd_scale
        for index, frame in enumerate(run_frames):
            deadline = started + index * period
            remaining = deadline - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
            right = tuple(
                origin + args.motion_scale * (float(value) - origin)
                for value, origin in zip(frame.right, right_origin)
            )
            if args.j12_only:
                right = right[:2] + right_origin[2:]
            robot.submit_raw_mit(
                left_positions=frame.left,
                right_positions=right,
                kp=kp,
                kd=kd,
            )
            submitted_frames += 1
            submission_events.append(
                {"monotonic_ns": time.monotonic_ns(), "frame_index": index}
            )
        elapsed = time.perf_counter() - started
        result.update(
            {
                "submitted_frames": len(run_frames),
                "elapsed_s": elapsed,
                "submit_hz": len(run_frames) / elapsed,
                "runtime_health": _jsonable(robot.safety_health),
                "motor_stats_before": before,
                "motor_stats_after": _motor_stats(robot),
            }
        )
    except BaseException as exc:
        failure = exc
        result["failure"] = f"{type(exc).__name__}: {exc}"
        result["submitted_frames"] = submitted_frames
        if raw_started is not None:
            result["elapsed_s"] = time.perf_counter() - raw_started
            if result["elapsed_s"] > 0:
                result["submit_hz"] = submitted_frames / result["elapsed_s"]
        try:
            result["runtime_health"] = _jsonable(robot.safety_health)
            result["motor_stats_after"] = _motor_stats(robot)
        except Exception as diagnostic_exc:
            result["post_fault_diagnostic_error"] = (
                f"{type(diagnostic_exc).__name__}: {diagnostic_exc}"
            )
    finally:
        if vbus_sampler is not None:
            vbus_sampler.stop()
            result["vbus_sampling"] = vbus_sampler.summary()
        try:
            if robot.connected:
                robot.disable()
                result["health_after_disable"] = _jsonable(robot.safety_health)
        except Exception as exc:
            result["disable_error"] = f"{type(exc).__name__}: {exc}"
        try:
            robot.close()
        except Exception as exc:
            result["close_error"] = f"{type(exc).__name__}: {exc}"
        capture.stop()
        result["can_summary"] = capture.summary()
        result["raw_mailbox"] = capture.mailbox_summary(submission_events)
        result["finished_wall_time_ns"] = time.time_ns()
        capture.dump(args.output, result, submission_events)

    printable = {
        key: result.get(key)
        for key in (
            "source_targets",
            "generated_frames",
            "requested_control_hz",
            "submitted_frames",
            "elapsed_s",
            "submit_hz",
            "failure",
            "disable_error",
            "close_error",
        )
        if key in result
    }
    if "runtime_health" in result:
        health = result["runtime_health"]
        printable["runtime"] = {
            "state": health["state"],
            "fault_reason": health["fault_reason"],
            "left_transport": health["left_transport"],
            "right_transport": health["right_transport"],
        }
    if "disable_report" in result:
        report = result["disable_report"]
        printable["disable"] = {
            key: report[key]
            for key in (
                "success",
                "expected_count",
                "disabled_count",
                "missing_count",
                "failure_count",
            )
        }
    printable["can_summary"] = result["can_summary"]
    printable["raw_mailbox"] = result["raw_mailbox"]
    print(json.dumps(printable, indent=2))
    print(f"diagnostic={args.output}")
    if failure is not None:
        raise failure
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Replay a recorded Yunyi VR command stream on real hardware and "
            "capture a bounded in-memory SocketCAN diagnostic trace."
        )
    )
    parser.add_argument(
        "--trace",
        type=Path,
        default=DEFAULT_TRACE,
        help="VR JSONL input (or set ARX_D_CAN_VR_TRACE)",
    )
    parser.add_argument("--control-hz", type=int, choices=(400, 500), default=500)
    parser.add_argument("--history-seconds", type=float, default=5.0)
    parser.add_argument("--start-timeout-s", type=float, default=15.0)
    parser.add_argument(
        "--start-velocity", type=float, default=5.0,
        help="普通位置接口起步速度档位，范围 0～100",
    )
    parser.add_argument("--j12-only", action="store_true")
    parser.add_argument("--preposition-only", action="store_true")
    parser.add_argument("--motion-scale", type=float, default=1.0)
    parser.add_argument("--j12-kp-scale", type=float, default=1.0)
    parser.add_argument("--j12-kd-scale", type=float, default=1.0)
    parser.add_argument("--feedback-max-age-ms", type=float, default=300.0)
    parser.add_argument("--feedback-failure-threshold", type=int, default=3)
    parser.add_argument("--feedback-check-hz", type=int, default=100)
    parser.add_argument(
        "--vbus-sample-hz",
        type=float,
        default=0.0,
        help="read register 60 from right J1/J2 at this sweep rate (diagnostic traffic)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/dev/shm/yunyi_vr_raw_mit_diagnostic.jsonl"),
    )
    return run(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
