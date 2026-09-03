from __future__ import annotations

import queue
import threading
from types import SimpleNamespace

import pytest

from arx_d_can._dds.client import DdsRuntimeClient, _cyclone_xml
from arx_d_can._dds.errors import RuntimeCallError
from arx_d_can._dds.models import RuntimeControlMode, SafetyState
from arx_d_can._dds.types import (
    PROTOCOL_MAJOR,
    PROTOCOL_MINOR,
    ControlOperation,
    ControlReply,
    ControlRequest,
    Discovery,
    ProtocolError,
    StreamCommand,
    StreamKind,
)


def test_wire_enums_and_type_names_match_cpp_idl() -> None:
    assert PROTOCOL_MAJOR == 1
    assert PROTOCOL_MINOR == 0
    assert ProtocolError.OK.value == 0
    assert ProtocolError.INTERNAL_ERROR.value == 9
    assert ControlOperation.ACQUIRE_LEASE.value == 0
    assert ControlOperation.GET_BIMANUAL_STATUS.value == 37
    assert StreamKind.PV.value == 0
    assert StreamKind.MIT_FAST.value == 2
    assert Discovery.__idl_typename__ == "articore_wire.Discovery"
    assert ControlRequest.__idl_typename__ == "articore_wire.ControlRequest"


def test_fixed_wire_types_serialize() -> None:
    request = ControlRequest(
        1,
        0,
        "yunyi-001",
        "pytest",
        "",
        1,
        2,
        3,
        ControlOperation.ENABLE,
        0,
        2,
        [0.0] * 8,
        [0.0] * 14,
        [0.0] * 6,
        [0.0] * 6,
        [0.0] * 6,
    )
    stream = StreamCommand(
        1,
        0,
        "yunyi-001",
        "pytest",
        "",
        4,
        3,
        StreamKind.MIT,
        [0.0] * 14,
        [0.0] * 14,
        [10.0] * 14,
        [1.0] * 14,
        [0.0] * 14,
        50.0,
    )

    assert ControlRequest.__idl__.deserialize(
        ControlRequest.__idl__.serialize(request)
    ) == request
    assert StreamCommand.__idl__.deserialize(
        StreamCommand.__idl__.serialize(stream)
    ) == stream


def test_explicit_interfaces_and_peer_produce_cyclone_config() -> None:
    xml = _cyclone_xml(7, ("eth0", "enp2s0"), ("192.168.1.185",))
    assert '<Domain Id="7">' in xml
    assert '<NetworkInterface name="eth0" priority="10"/>' in xml
    assert '<NetworkInterface name="enp2s0" priority="0"/>' in xml
    assert '<Peer Address="192.168.1.185"/>' in xml
    with pytest.raises(ValueError):
        _cyclone_xml(0, ())
    with pytest.raises(ValueError):
        _cyclone_xml(0, ("bad interface",))
    with pytest.raises(ValueError):
        _cyclone_xml(0, peers=("not-an-ip",))


def test_robot_ip_environment_default_is_validated(monkeypatch) -> None:
    monkeypatch.setenv("ARTICORE_ROBOT_IP", "not-an-ip")
    with pytest.raises(ValueError):
        DdsRuntimeClient(robot_id="yunyi-001")


def _request_client(reply_error: ProtocolError = ProtocolError.OK):
    client = DdsRuntimeClient.__new__(DdsRuntimeClient)
    client.robot_id = "yunyi-001"
    client.client_id = "pytest"
    client.security_identity = ""
    client.request_timeout = 0.1
    client._closed = False
    client._lease_id = 99
    client._sequence = 0
    client._request_id = 0
    client._write_lock = threading.Lock()
    client._cache_lock = threading.Lock()
    client._waiters = {}
    client._transport_error = None
    captured = []

    class Writer:
        def write(self, request: ControlRequest) -> None:
            captured.append(request)
            client._handle_sample(
                ControlReply(
                    1,
                    0,
                    request.robot_id,
                    request.client_id,
                    "",
                    1,
                    request.request_id,
                    request.lease_id,
                    reply_error,
                    "rejected" if reply_error is not ProtocolError.OK else "",
                    [0.0] * 64,
                )
            )

    client._request_writer = Writer()
    return client, captured


def test_request_correlates_reply_and_carries_lease_sequence() -> None:
    client, captured = _request_client()
    client._request(ControlOperation.SET_SPEED_PERCENT, scalar=(50.0,))

    assert len(captured) == 1
    assert captured[0].lease_id == 99
    assert captured[0].sequence_id == 1
    assert captured[0].request_id == 1
    assert captured[0].scalar == pytest.approx([50.0] + [0.0] * 7)


def test_protocol_error_is_exposed_as_stable_runtime_error_code() -> None:
    client, _ = _request_client(ProtocolError.NO_LEASE)
    with pytest.raises(RuntimeCallError) as raised:
        client._request(ControlOperation.ENABLE)
    assert raised.value.code == "NO_LEASE"


def test_request_validates_fixed_arrays_before_writing() -> None:
    client, captured = _request_client()
    with pytest.raises(ValueError):
        client._request(ControlOperation.SOLVE_IK, pose_a=(0.0,) * 5)
    assert captured == []


def _connect_client(state: SafetyState):
    client = DdsRuntimeClient.__new__(DdsRuntimeClient)
    client._closed = False
    client._lease_id = 0
    client._mode_configured = False
    client._maintenance_only = False
    client._with_grippers = True
    client.control_mode = RuntimeControlMode.PV
    client.discovery_timeout = 0.1
    client.domain_id = 0
    client.robot_id = "yunyi-001"
    client._cache_lock = threading.Lock()
    client._discovery = Discovery(
        PROTOCOL_MAJOR,
        PROTOCOL_MINOR,
        client.robot_id,
        "runtime",
        "",
        1,
        "1.0.2",
        0,
        True,
    )
    client._discovery_ready = threading.Event()
    operations = []

    def request(operation, **_kwargs):
        operations.append(operation)
        if operation is ControlOperation.ACQUIRE_LEASE:
            return SimpleNamespace(lease_id=77, values=[0.0] * 64)
        values = [0.0] * 64
        if operation is ControlOperation.QUERY_HEALTH:
            values[0] = float(state)
        if operation is ControlOperation.HAS_GRIPPERS:
            values[0] = 1.0
        return SimpleNamespace(lease_id=77, values=values)

    client._request = request
    client._start_heartbeat = lambda: operations.append("START_HEARTBEAT")
    client._stop_heartbeat = lambda: operations.append("STOP_HEARTBEAT")
    return client, operations


def test_connect_keeps_faulted_runtime_as_maintenance_session() -> None:
    client, operations = _connect_client(SafetyState.FAULT)

    client.connect()

    assert client.connected
    assert not client._mode_configured
    assert operations == [
        ControlOperation.ACQUIRE_LEASE,
        "START_HEARTBEAT",
        ControlOperation.QUERY_HEALTH,
        ControlOperation.HAS_GRIPPERS,
    ]


def test_connect_configures_ready_runtime_after_heartbeat_starts() -> None:
    client, operations = _connect_client(SafetyState.READY)

    client.connect()

    assert client._mode_configured
    assert operations == [
        ControlOperation.ACQUIRE_LEASE,
        "START_HEARTBEAT",
        ControlOperation.QUERY_HEALTH,
        ControlOperation.CONFIGURE_MODE,
        ControlOperation.HAS_GRIPPERS,
    ]


def test_explicit_maintenance_connect_skips_mode_even_when_ready() -> None:
    client, operations = _connect_client(SafetyState.READY)

    client.connect(maintenance=True)

    assert client.connected
    assert client._maintenance_only
    assert not client._mode_configured
    assert operations == [
        ControlOperation.ACQUIRE_LEASE,
        "START_HEARTBEAT",
        ControlOperation.QUERY_HEALTH,
        ControlOperation.HAS_GRIPPERS,
    ]


def test_clear_faults_configures_requested_mode_only_after_success() -> None:
    client, operations = _connect_client(SafetyState.FAULT)
    client._lease_id = 77

    client.clear_faults()

    assert operations == [
        ControlOperation.CLEAR_FAULTS,
        ControlOperation.CONFIGURE_MODE,
    ]
    assert client._mode_configured


def test_maintenance_clear_faults_never_configures_mode() -> None:
    client, operations = _connect_client(SafetyState.READY)
    client._lease_id = 77
    client._maintenance_only = True

    client.clear_faults()

    assert operations == [ControlOperation.CLEAR_FAULTS]
    assert not client._mode_configured


def test_failed_clear_faults_does_not_configure_enable_or_move() -> None:
    client, operations = _connect_client(SafetyState.FAULT)
    client._lease_id = 77
    client._mode_configured = True

    def reject_clear(operation, **_kwargs):
        operations.append(operation)
        if operation is ControlOperation.CLEAR_FAULTS:
            raise RuntimeCallError(
                "CLEAR_FAULTS rejected: current_state=FAULT, "
                "fault_reason=emergency stop requested",
                code="WRONG_STATE",
            )
        return SimpleNamespace(lease_id=77, values=[0.0] * 64)

    client._request = reject_clear
    with pytest.raises(RuntimeCallError, match="emergency stop requested"):
        client.clear_faults()

    assert operations == [ControlOperation.CLEAR_FAULTS]
    assert not client._mode_configured
