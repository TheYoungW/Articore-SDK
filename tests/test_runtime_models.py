from arx_d_can._motor_abi import RuntimeOperation


def test_runtime_operation_values_match_the_native_cartesian_abi() -> None:
    assert RuntimeOperation.MOVE_JOINT_TRAJECTORY == 10
    assert RuntimeOperation.CANCEL_MOTION == 11
    assert RuntimeOperation.SET_POSE == 12
    assert RuntimeOperation.CANCEL_ALL_MOTIONS == 13
    assert RuntimeOperation.MOVE_LINEAR_TRAJECTORY == 14
    assert RuntimeOperation.MOVE_CIRCULAR_TRAJECTORY == 15
