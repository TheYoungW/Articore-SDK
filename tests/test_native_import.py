from arx_d_can._motor_abi import (
    ArticoreRuntime,
    GravityCompensationPhase,
    RuntimeControlMode,
    SafetyState,
)


def test_import_symbols() -> None:
    assert ArticoreRuntime is not None
    assert RuntimeControlMode.MIT.value == 2
    assert GravityCompensationPhase.ACTIVE.value == 2
    assert SafetyState.DISCONNECTED.value == 0
