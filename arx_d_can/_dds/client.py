"""Thin Cyclone DDS client for the RK3588 product Runtime service."""
from __future__ import annotations

from collections import deque
import ipaddress
import math
import os
import queue
import re
import threading
import time
import uuid
from typing import Sequence

from cyclonedds.core import (
    InstanceState,
    ReadCondition,
    SampleState,
    ViewState,
    WaitSet,
)
from cyclonedds.domain import Domain, DomainParticipant
from cyclonedds.pub import DataWriter
from cyclonedds.qos import Policy, Qos
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.util import duration

from .errors import RuntimeCallError
from .models import (
    BimanualFollowPhase,
    BimanualFollowStatus,
    FeedbackIssueScope,
    GravityCompensationPhase,
    GravityCompensationStatus,
    JointLimit,
    ProductArmState,
    ProductGripperState,
    ProductPose,
    ProductState,
    RuntimeControlMode,
    RuntimeTransportHealth,
    SafetyHealth,
    SafetyState,
)
from .types import (
    CONTROL_REPLY_TOPIC,
    CONTROL_REQUEST_TOPIC,
    DISCOVERY_TOPIC,
    HEALTH_TOPIC,
    MOTION_EVENT_TOPIC,
    PROTOCOL_MAJOR,
    PROTOCOL_MINOR,
    STATE_TOPIC,
    STREAM_COMMAND_TOPIC,
    ControlOperation,
    ControlReply,
    ControlRequest,
    Discovery,
    Health,
    MotionEvent,
    ProtocolError,
    RobotState,
    StreamCommand,
    StreamKind,
)


_INTERFACE = re.compile(r"^[A-Za-z0-9_.:-]{1,15}$")


def _reliable(depth: int, *, transient: bool = False) -> Qos:
    policies = [
        Policy.Reliability.Reliable(max_blocking_time=duration(milliseconds=20)),
        Policy.History.KeepLast(depth),
    ]
    if transient:
        policies.append(Policy.Durability.TransientLocal)
    return Qos(*policies)


def _best_effort(depth: int, *, lifespan_ms: int | None = None) -> Qos:
    policies = [Policy.Reliability.BestEffort, Policy.History.KeepLast(depth)]
    if lifespan_ms is not None:
        policies.append(Policy.Lifespan(duration(milliseconds=lifespan_ms)))
    return Qos(*policies)


def _cyclone_xml(
    domain_id: int,
    interfaces: Sequence[str] = (),
    peers: Sequence[str] = (),
) -> str:
    selected = tuple(str(value) for value in interfaces)
    if any(not _INTERFACE.fullmatch(value) for value in selected):
        raise ValueError("invalid DDS network interface name")
    selected_peers = tuple(str(ipaddress.ip_address(value)) for value in peers)
    if not selected and not selected_peers:
        raise ValueError("at least one DDS interface or peer must be configured")
    nodes = "".join(
        f'<NetworkInterface name="{name}" priority="{10 if index == 0 else 0}"/>'
        for index, name in enumerate(selected)
    )
    general = "<General>"
    if nodes:
        general += f"<Interfaces>{nodes}</Interfaces>"
    general += "<AllowMulticast>true</AllowMulticast></General>"
    discovery = ""
    if selected_peers:
        peer_nodes = "".join(
            f'<Peer Address="{address}"/>' for address in selected_peers
        )
        discovery = f"<Discovery><Peers>{peer_nodes}</Peers></Discovery>"
    return f'<CycloneDDS><Domain Id="{int(domain_id)}">{general}{discovery}</Domain></CycloneDDS>'


def _fixed(values: Sequence[float], size: int, name: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != size:
        raise ValueError(f"{name} must contain exactly {size} values")
    if any(not math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite values")
    return result


def _empty_transport() -> RuntimeTransportHealth:
    return RuntimeTransportHealth(
        connected=False,
        healthy=False,
        consecutive_send_failures=0,
        consecutive_feedback_failures=0,
        last_feedback_age_ns=None,
        tx_frames=0,
        rx_frames=0,
        send_errors=0,
        receive_errors=0,
        last_tx_age_ns=None,
        last_rx_age_ns=None,
        last_error=None,
    )


def _initial_health() -> SafetyHealth:
    return SafetyHealth(
        state=SafetyState.DISCONNECTED,
        safe_holding=False,
        disable_confirmed=False,
        last_successful_command_age_ns=None,
        last_fresh_feedback_age_ns=None,
        consecutive_send_failures=0,
        consecutive_feedback_failures=0,
        left_transport=_empty_transport(),
        right_transport=_empty_transport(),
        grippers=(),
        motor_faults=(),
        unconfirmed_disable=(),
        fault_reason=None,
    )


class DdsRuntimeClient:
    """One robot/client DDS session; Runtime logic remains on the RK3588."""

    def __init__(
        self,
        *,
        robot_id: str,
        domain_id: int = 0,
        client_id: str | None = None,
        security_identity: str = "",
        robot_ip: str | None = None,
        network_interfaces: Sequence[str] | None = None,
        request_timeout: float = 1.0,
        discovery_timeout: float = 5.0,
        control_mode: RuntimeControlMode = RuntimeControlMode.MIT,
        with_grippers: bool = True,
    ) -> None:
        if not robot_id or len(robot_id.encode()) > 63:
            raise ValueError("robot_id must contain 1..63 UTF-8 bytes")
        selected_client = client_id or f"python-{uuid.uuid4().hex[:12]}"
        if not selected_client or len(selected_client.encode()) > 63:
            raise ValueError("client_id must contain 1..63 UTF-8 bytes")
        if len(security_identity.encode()) > 127:
            raise ValueError("security_identity must contain at most 127 UTF-8 bytes")
        if request_timeout <= 0.0 or discovery_timeout <= 0.0:
            raise ValueError("timeouts must be positive")

        self.robot_id = robot_id
        self.domain_id = int(domain_id)
        if not 0 <= self.domain_id <= 0xFFFFFFFF:
            raise ValueError("domain_id must be in the range 0..4294967295")
        self.client_id = selected_client
        self.security_identity = security_identity
        selected_robot_ip = robot_ip or os.environ.get("ARTICORE_ROBOT_IP")
        self.robot_ip = (
            str(ipaddress.ip_address(selected_robot_ip))
            if selected_robot_ip
            else None
        )
        self.request_timeout = float(request_timeout)
        self.discovery_timeout = float(discovery_timeout)
        self.control_mode = control_mode
        self._mode_configured = False
        self._maintenance_only = False
        self._with_grippers = bool(with_grippers)
        self._lease_id = 0
        self._sequence = 0
        self._request_id = 0
        self._write_lock = threading.Lock()
        self._cache_lock = threading.Lock()
        self._waiters: dict[int, queue.Queue[ControlReply]] = {}
        self._discovery: Discovery | None = None
        self._state: RobotState | None = None
        self._health_wire: Health | None = None
        self._health = _initial_health()
        self._motion_events: dict[int, MotionEvent] = {}
        self._state_arrivals: deque[float] = deque()
        self._transport_error: str | None = None
        self._closed = False
        self._stop = threading.Event()
        self._heartbeat_stop = threading.Event()
        self._discovery_ready = threading.Event()

        self._domain: Domain | None = None
        if network_interfaces is not None or self.robot_ip is not None:
            self._domain = Domain(
                self.domain_id,
                _cyclone_xml(
                    self.domain_id,
                    network_interfaces or (),
                    (self.robot_ip,) if self.robot_ip else (),
                ),
            )
        self._participant = DomainParticipant(self.domain_id)
        self._create_entities()
        self._reader_thread = threading.Thread(
            target=self._read_loop, name="articore-dds-reader", daemon=True
        )
        self._reader_thread.start()
        self._heartbeat_thread: threading.Thread | None = None

    def _create_entities(self) -> None:
        participant = self._participant
        self._discovery_reader = DataReader(
            participant,
            Topic(participant, DISCOVERY_TOPIC, Discovery),
            _reliable(1, transient=True),
        )
        request_topic = Topic(participant, CONTROL_REQUEST_TOPIC, ControlRequest)
        self._request_writer = DataWriter(participant, request_topic, _reliable(32))
        self._reply_reader = DataReader(
            participant,
            Topic(participant, CONTROL_REPLY_TOPIC, ControlReply),
            _reliable(32),
        )
        self._stream_writer = DataWriter(
            participant,
            Topic(participant, STREAM_COMMAND_TOPIC, StreamCommand),
            _best_effort(1, lifespan_ms=20),
        )
        self._state_reader = DataReader(
            participant,
            Topic(participant, STATE_TOPIC, RobotState),
            _best_effort(1),
        )
        self._health_reader = DataReader(
            participant,
            Topic(participant, HEALTH_TOPIC, Health),
            _reliable(1, transient=True),
        )
        self._motion_reader = DataReader(
            participant,
            Topic(participant, MOTION_EVENT_TOPIC, MotionEvent),
            _reliable(32),
        )
        self._readers = (
            self._discovery_reader,
            self._reply_reader,
            self._state_reader,
            self._health_reader,
            self._motion_reader,
        )
        mask = SampleState.Any | ViewState.Any | InstanceState.Any
        self._conditions = tuple(ReadCondition(reader, mask) for reader in self._readers)
        self._waitset = WaitSet(participant)
        for condition in self._conditions:
            self._waitset.attach(condition)

    @property
    def closed(self) -> bool:
        return self._closed

    @property
    def connected(self) -> bool:
        return not self._closed and self._lease_id != 0

    @property
    def has_grippers(self) -> bool:
        return self._with_grippers

    def _next_sequence_locked(self) -> int:
        self._sequence += 1
        return self._sequence

    def _next_request_locked(self) -> int:
        self._request_id += 1
        return self._request_id

    def _request(
        self,
        operation: ControlOperation,
        *,
        side: int = 0,
        mode: int = 0,
        scalar: Sequence[float] = (),
        positions: Sequence[float] = (),
        pose_a: Sequence[float] = (),
        pose_b: Sequence[float] = (),
        pose_c: Sequence[float] = (),
        timeout: float | None = None,
        lease_id: int | None = None,
    ) -> ControlReply:
        if self._closed:
            raise RuntimeCallError("DDS client is closed", code="TRANSPORT_ERROR")
        values = [float(value) for value in scalar]
        if len(values) > 8 or any(not math.isfinite(value) for value in values):
            raise ValueError("scalar must contain at most 8 finite values")
        values.extend([0.0] * (8 - len(values)))
        frame = [0.0] * 14 if not positions else _fixed(positions, 14, "positions")
        poses = [
            [0.0] * 6 if not value else _fixed(value, 6, name)
            for value, name in (
                (pose_a, "pose_a"),
                (pose_b, "pose_b"),
                (pose_c, "pose_c"),
            )
        ]
        response_queue: queue.Queue[ControlReply] = queue.Queue(maxsize=1)
        with self._write_lock:
            request_id = self._next_request_locked()
            request = ControlRequest(
                PROTOCOL_MAJOR,
                PROTOCOL_MINOR,
                self.robot_id,
                self.client_id,
                self.security_identity,
                self._next_sequence_locked(),
                request_id,
                self._lease_id if lease_id is None else int(lease_id),
                operation,
                int(side),
                int(mode),
                values,
                frame,
                poses[0],
                poses[1],
                poses[2],
            )
            with self._cache_lock:
                self._waiters[request_id] = response_queue
            try:
                self._request_writer.write(request)
            except Exception as error:
                with self._cache_lock:
                    self._waiters.pop(request_id, None)
                raise RuntimeCallError(
                    f"failed to publish DDS request: {error}",
                    code="TRANSPORT_ERROR",
                ) from error
        try:
            reply = response_queue.get(timeout=timeout or self.request_timeout)
        except queue.Empty as error:
            with self._cache_lock:
                self._waiters.pop(request_id, None)
            detail = self._transport_error or f"DDS request {operation.name} timed out"
            raise RuntimeCallError(detail, code="TIMEOUT") from error
        if reply.protocol_major != PROTOCOL_MAJOR:
            raise RuntimeCallError(
                "Runtime protocol major version does not match SDK",
                code="VERSION_MISMATCH",
            )
        if reply.error is not ProtocolError.OK:
            raise RuntimeCallError(
                reply.message or f"Runtime rejected {operation.name}: {reply.error.name}",
                code=reply.error.name,
            )
        return reply

    def _stream(
        self,
        kind: StreamKind,
        *,
        positions: Sequence[float],
        velocities: Sequence[float] = (0.0,) * 14,
        kp: Sequence[float] = (0.0,) * 14,
        kd: Sequence[float] = (0.0,) * 14,
        feedforward_torques: Sequence[float] = (0.0,) * 14,
        speed_percent: float = 0.0,
    ) -> None:
        if not self.connected:
            raise RuntimeCallError("a control lease is required", code="NO_LEASE")
        speed = float(speed_percent)
        if not math.isfinite(speed) or not 0.0 <= speed <= 100.0:
            raise ValueError("speed_percent must be finite and in the range 0..100")
        if kind is StreamKind.PV and speed < 1.0:
            raise ValueError("PV speed_percent must be in the range 1..100")
        with self._write_lock:
            sample = StreamCommand(
                PROTOCOL_MAJOR,
                PROTOCOL_MINOR,
                self.robot_id,
                self.client_id,
                self.security_identity,
                self._next_sequence_locked(),
                self._lease_id,
                kind,
                _fixed(positions, 14, "positions"),
                _fixed(velocities, 14, "velocities"),
                _fixed(kp, 14, "kp"),
                _fixed(kd, 14, "kd"),
                _fixed(feedforward_torques, 14, "feedforward_torques"),
                speed,
            )
            try:
                self._stream_writer.write(sample)
            except Exception as error:
                raise RuntimeCallError(
                    f"failed to publish DDS stream command: {error}",
                    code="TRANSPORT_ERROR",
                ) from error

    def _read_loop(self) -> None:
        try:
            while not self._stop.is_set():
                self._waitset.wait(duration(milliseconds=50))
                for reader in self._readers:
                    for sample in reader.take(64):
                        self._handle_sample(sample)
        except Exception as error:
            self._transport_error = str(error)

    def _handle_sample(self, sample: object) -> None:
        if getattr(sample, "robot_id", None) != self.robot_id:
            return
        if isinstance(sample, Discovery):
            with self._cache_lock:
                self._discovery = sample
            self._discovery_ready.set()
        elif isinstance(sample, ControlReply):
            if sample.client_id != self.client_id:
                return
            with self._cache_lock:
                waiter = self._waiters.pop(sample.request_id, None)
            if waiter is not None:
                waiter.put_nowait(sample)
        elif isinstance(sample, RobotState):
            if sample.protocol_major != PROTOCOL_MAJOR:
                return
            now = time.monotonic()
            with self._cache_lock:
                self._state = sample
                self._state_arrivals.append(now)
                while self._state_arrivals and self._state_arrivals[0] < now - 1.0:
                    self._state_arrivals.popleft()
        elif isinstance(sample, Health):
            if sample.protocol_major != PROTOCOL_MAJOR:
                return
            converted = self._convert_health(sample)
            with self._cache_lock:
                self._health_wire = sample
                self._health = converted
        elif isinstance(sample, MotionEvent) and sample.client_id == self.client_id:
            with self._cache_lock:
                self._motion_events[sample.request_id] = sample

    def connect(self, *, maintenance: bool = False) -> None:
        if self.connected:
            if bool(maintenance) != self._maintenance_only:
                raise RuntimeCallError(
                    "disconnect before changing the session maintenance mode",
                    code="WRONG_STATE",
                )
            return
        if self._closed:
            raise RuntimeCallError("DDS client is closed", code="TRANSPORT_ERROR")
        deadline = time.monotonic() + self.discovery_timeout
        discovery = None
        while time.monotonic() < deadline:
            with self._cache_lock:
                discovery = self._discovery
            if discovery is not None:
                if discovery.protocol_major != PROTOCOL_MAJOR:
                    raise RuntimeCallError(
                        "Runtime protocol major version mismatch",
                        code="VERSION_MISMATCH",
                    )
                if discovery.ready:
                    break
            self._discovery_ready.clear()
            self._discovery_ready.wait(min(0.1, max(0.0, deadline - time.monotonic())))
        else:
            detail = (
                "Runtime service was discovered but CAN is not ready"
                if discovery is not None
                else f"robot {self.robot_id!r} was not discovered on domain {self.domain_id}"
            )
            raise RuntimeCallError(detail, code="TIMEOUT")
        reply = self._request(ControlOperation.ACQUIRE_LEASE, lease_id=0)
        self._lease_id = int(reply.lease_id)
        self._maintenance_only = bool(maintenance)
        try:
            self._start_heartbeat()
            runtime_state = self._query_runtime_state()
            if self._maintenance_only or runtime_state is SafetyState.FAULT:
                # A faulted Runtime is still a valid maintenance endpoint.
                # Explicit maintenance sessions also skip mode configuration
                # when Runtime is READY. Keep the lease alive so maintenance
                # operations can be sent without enabling or moving anything.
                self._mode_configured = False
            else:
                self._request(
                    ControlOperation.CONFIGURE_MODE,
                    mode=int(self.control_mode),
                )
                self._mode_configured = True
            grippers = self._request(ControlOperation.HAS_GRIPPERS)
            self._with_grippers = bool(round(grippers.values[0]))
        except Exception:
            self._stop_heartbeat()
            try:
                if self._lease_id:
                    self._request(ControlOperation.RELEASE_LEASE)
            finally:
                self._lease_id = 0
                self._mode_configured = False
                self._maintenance_only = False
            raise

    def _query_runtime_state(self) -> SafetyState:
        reply = self._request(ControlOperation.QUERY_HEALTH)
        try:
            return SafetyState(round(reply.values[0]))
        except (IndexError, TypeError, ValueError) as error:
            raise RuntimeCallError(
                "Runtime returned an invalid safety state",
                code="INTERNAL_ERROR",
            ) from error

    def _start_heartbeat(self) -> None:
        self._heartbeat_stop.clear()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, name="articore-dds-heartbeat", daemon=True
        )
        self._heartbeat_thread.start()

    def _stop_heartbeat(self) -> None:
        self._heartbeat_stop.set()
        if self._heartbeat_thread is not None:
            self._heartbeat_thread.join(timeout=0.3)
            self._heartbeat_thread = None

    def _heartbeat_loop(self) -> None:
        failures = 0
        while not self._heartbeat_stop.wait(0.05):
            if self._lease_id == 0:
                return
            try:
                self._request(
                    ControlOperation.HEARTBEAT,
                    timeout=min(self.request_timeout, 0.15),
                )
                failures = 0
            except RuntimeCallError:
                failures += 1
                if failures >= 2:
                    self._lease_id = 0
                    self._mode_configured = False
                    return

    def disconnect(self) -> None:
        if self._closed:
            return
        self._stop_heartbeat()
        if self._lease_id:
            try:
                self._request(ControlOperation.RELEASE_LEASE)
            except RuntimeCallError:
                pass
            self._lease_id = 0
            self._mode_configured = False
        self._maintenance_only = False
        self.close()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        self._reader_thread.join(timeout=0.2)
        try:
            self._participant._delete()
        except Exception:
            pass

    @property
    def state(self) -> ProductState:
        with self._cache_lock:
            sample = self._state
        if sample is None:
            raise RuntimeCallError("no robot state has been received", code="TIMEOUT")

        def arm(offset: int) -> ProductArmState:
            indexes = range(offset, offset + 7)
            return ProductArmState(
                positions=tuple(sample.positions[index] for index in indexes),
                velocities=tuple(sample.velocities[index] for index in indexes),
                torques=tuple(sample.torques[index] for index in indexes),
                enabled=tuple(
                    bool(sample.enabled_mask & (1 << index))
                    if sample.enabled_valid_mask & (1 << index)
                    else None
                    for index in indexes
                ),
                mos_temperatures=tuple(
                    sample.mos_temperatures[index]
                    if sample.temperature_valid_mask & (1 << index)
                    else None
                    for index in indexes
                ),
                rotor_temperatures=tuple(
                    sample.rotor_temperatures[index]
                    if sample.temperature_valid_mask & (1 << index)
                    else None
                    for index in indexes
                ),
            )

        unavailable = ProductGripperState(False, 0.0, 0)
        return ProductState(
            has_grippers=self._with_grippers,
            left=arm(0),
            right=arm(7),
            left_gripper=unavailable if self._with_grippers else None,
            right_gripper=unavailable if self._with_grippers else None,
            motion_arrived=sample.motion_arrived,
            timestamp_ns=int(sample.source_timestamp_ns),
            sequence=int(sample.sequence_id),
        )

    @property
    def health(self) -> SafetyHealth:
        with self._cache_lock:
            return self._health

    def _convert_health(self, sample: Health) -> SafetyHealth:
        try:
            state = SafetyState(int(sample.state))
        except ValueError:
            state = SafetyState.FAULT
        transport = RuntimeTransportHealth(
            connected=state is not SafetyState.DISCONNECTED,
            healthy=not sample.degraded and not sample.safe_stopped,
            consecutive_send_failures=int(sample.consecutive_send_failures),
            consecutive_feedback_failures=int(sample.consecutive_feedback_failures),
            last_feedback_age_ns=None,
            tx_frames=0,
            rx_frames=0,
            send_errors=0,
            receive_errors=0,
            last_tx_age_ns=None,
            last_rx_age_ns=None,
            last_error=sample.fault_reason or None,
        )
        return SafetyHealth(
            state=state,
            safe_holding=sample.safe_holding,
            disable_confirmed=sample.disable_confirmed,
            last_successful_command_age_ns=None,
            last_fresh_feedback_age_ns=None,
            consecutive_send_failures=int(sample.consecutive_send_failures),
            consecutive_feedback_failures=int(sample.consecutive_feedback_failures),
            left_transport=transport,
            right_transport=transport,
            grippers=(),
            motor_faults=(),
            unconfirmed_disable=(),
            fault_reason=sample.fault_reason or None,
            degraded=sample.degraded,
            safe_stopped=sample.safe_stopped,
            requires_resynchronization=sample.requires_resynchronization,
            safety_reason=sample.safety_reason or None,
            feedback_issue_scope=FeedbackIssueScope.NONE,
        )

    def get_fps(self) -> float:
        now = time.monotonic()
        with self._cache_lock:
            while self._state_arrivals and self._state_arrivals[0] < now - 1.0:
                self._state_arrivals.popleft()
            arrivals = tuple(self._state_arrivals)
        if len(arrivals) < 2:
            return 0.0
        return (len(arrivals) - 1) / (arrivals[-1] - arrivals[0])

    def enable(self, motors: Sequence[str] | None = None) -> bool:
        if motors is not None:
            raise ValueError("DDS v1 only supports atomic whole-robot enable")
        self._request(ControlOperation.ENABLE)
        return True

    def disable(self, motors: Sequence[str] | None = None) -> bool:
        if motors is not None:
            raise ValueError("DDS v1 only supports atomic whole-robot disable")
        self._request(ControlOperation.DISABLE)
        return True

    def configure_mode(self, mode: RuntimeControlMode) -> None:
        self._mode_configured = False
        self._request(ControlOperation.CONFIGURE_MODE, mode=int(mode))
        self.control_mode = mode
        self._mode_configured = True

    def set_joint_pv(self, positions: Sequence[float], speed_percent: float) -> None:
        self._stream(StreamKind.PV, positions=positions, speed_percent=speed_percent)

    def set_joint_mit_fast(self, positions: Sequence[float], speed_percent: float) -> None:
        self._stream(StreamKind.MIT_FAST, positions=positions, speed_percent=speed_percent)

    def set_joint_mit(
        self,
        positions: Sequence[float],
        velocities: Sequence[float],
        kp: Sequence[float],
        kd: Sequence[float],
        feedforward_torques: Sequence[float],
    ) -> None:
        self._stream(
            StreamKind.MIT,
            positions=positions,
            velocities=velocities,
            kp=kp,
            kd=kd,
            feedforward_torques=feedforward_torques,
        )

    def set_speed_percent(self, value: float) -> None:
        self._request(ControlOperation.SET_SPEED_PERCENT, scalar=(value,))

    def get_speed_percent(self) -> float:
        return float(self._request(ControlOperation.GET_SPEED_PERCENT).values[0])

    def set_max_speed(self, value: float) -> None:
        self._request(ControlOperation.SET_MAX_SPEED, scalar=(value,))

    def get_max_speed(self) -> float:
        return float(self._request(ControlOperation.GET_MAX_SPEED).values[0])

    def set_max_acceleration(self, value: float) -> None:
        self._request(ControlOperation.SET_MAX_ACCELERATION, scalar=(value,))

    def get_max_acceleration(self) -> float:
        return float(self._request(ControlOperation.GET_MAX_ACCELERATION).values[0])

    def get_joint_limits(self) -> tuple[JointLimit, ...]:
        values = self._request(ControlOperation.GET_JOINT_LIMITS).values
        return tuple(
            JointLimit(values[index], values[14 + index], values[28 + index])
            for index in range(14)
        )

    def get_pose(self, side: int) -> list[float]:
        return list(self._request(ControlOperation.GET_POSE, side=side).values[:6])

    def get_pose_sample(self, side: int) -> ProductPose:
        values = tuple(self.get_pose(side))
        with self._cache_lock:
            state = self._state
        return ProductPose(
            side,
            values,  # type: ignore[arg-type]
            int(state.source_timestamp_ns) if state else 0,
            int(state.sequence_id) if state else 0,
        )

    def set_tcp_offset(self, side: int, offset: Sequence[float]) -> None:
        self._request(ControlOperation.SET_TCP_OFFSET, side=side, pose_a=offset)

    def get_tcp_offset(self, side: int) -> list[float]:
        return list(self._request(ControlOperation.GET_TCP_OFFSET, side=side).values[:6])

    def reset_tcp_offset(self, side: int) -> None:
        self._request(ControlOperation.RESET_TCP_OFFSET, side=side)

    def solve_ik(
        self, left_target_pose: Sequence[float], right_target_pose: Sequence[float]
    ) -> tuple[float, ...]:
        return tuple(
            self._request(
                ControlOperation.SOLVE_IK,
                pose_a=left_target_pose,
                pose_b=right_target_pose,
            ).values[:14]
        )

    def move_pose(self, side: int, target_pose: Sequence[float]) -> None:
        self._request(ControlOperation.MOVE_POSE, side=side, pose_a=target_pose)

    def move_linear(
        self,
        side: int,
        start_pose: Sequence[float] | None,
        end_pose: Sequence[float],
    ) -> None:
        self._request(
            ControlOperation.MOVE_LINEAR,
            side=side,
            pose_a=start_pose or (),
            pose_b=end_pose,
        )

    def move_circular(
        self,
        side: int,
        start_pose: Sequence[float],
        via_pose: Sequence[float],
        end_pose: Sequence[float],
    ) -> None:
        self._request(
            ControlOperation.MOVE_CIRCULAR,
            side=side,
            pose_a=start_pose,
            pose_b=via_pose,
            pose_c=end_pose,
        )

    def stop_motion(self) -> None:
        self._request(ControlOperation.STOP_MOTION)

    def set_product_grippers(
        self, *, left: float, right: float, gripper_level: int, mode: int
    ) -> None:
        self._request(
            ControlOperation.SET_GRIPPERS,
            scalar=(left, right, float(gripper_level), float(mode)),
        )

    def start_gravity_compensation(self, *, transition_ms: int = 0) -> None:
        self._request(
            ControlOperation.START_GRAVITY_COMPENSATION,
            scalar=(float(transition_ms),),
        )

    def stop_gravity_compensation(self) -> None:
        self._request(ControlOperation.STOP_GRAVITY_COMPENSATION)

    @property
    def gravity_compensation_status(self) -> GravityCompensationStatus:
        values = self._request(ControlOperation.GET_GRAVITY_STATUS).values
        return GravityCompensationStatus(
            phase=GravityCompensationPhase(round(values[0])),
            active=bool(round(values[1])),
            transition_progress=values[2],
            control_cycles=0,
            joint_count=14,
            gravity_feedforward_torque=tuple(values[3:17]),
        )

    def start_bimanual_follow(self, side: int) -> None:
        self._request(ControlOperation.START_BIMANUAL_FOLLOW, side=side)

    def stop_bimanual_follow(self) -> None:
        self._request(ControlOperation.STOP_BIMANUAL_FOLLOW)

    @property
    def bimanual_follow_status(self) -> BimanualFollowStatus:
        values = self._request(ControlOperation.GET_BIMANUAL_STATUS).values
        leader = "left" if round(values[2]) == 0 else "right"
        return BimanualFollowStatus(
            phase=BimanualFollowPhase(round(values[0])),
            active=bool(round(values[1])),
            leader=leader,
            follower="right" if leader == "left" else "left",
            transition_progress=values[3],
            control_cycles=0,
            leader_positions=tuple(values[5:12]),
            follower_target_positions=tuple(values[12:19]),
            max_tracking_error=values[4],
            error=None,
        )

    def estop(self) -> None:
        self._request(ControlOperation.ESTOP)

    def recover(self) -> None:
        self._request(ControlOperation.RECOVER)

    def set_zero(self) -> bool:
        self._request(ControlOperation.SET_ZERO)
        return True

    def clear_faults(self) -> None:
        self._mode_configured = False
        self._request(ControlOperation.CLEAR_FAULTS)
        if self._maintenance_only:
            return
        # CLEAR_FAULTS only restores a disabled READY Runtime. Configure the
        # caller's requested mode afterwards; never enable or move here.
        self.configure_mode(self.control_mode)


__all__ = ["DdsRuntimeClient"]
