from arx_d_can._motor_abi import RuntimeOperation


def test_runtime_operation_values_match_the_native_cartesian_abi() -> None:
    assert RuntimeOperation.START_TRAJECTORY == 11
    assert RuntimeOperation.CANCEL_TRAJECTORY == 12
    assert RuntimeOperation.MOVE_POSE == 13
    assert RuntimeOperation.CANCEL_CARTESIAN_MOTION == 14
    assert RuntimeOperation.MOVE_LINEAR == 15
    assert RuntimeOperation.MOVE_CIRCULAR == 16
