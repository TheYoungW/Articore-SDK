from arx_d_can._dds import RuntimeOperation


def test_runtime_operation_values_match_the_native_cartesian_abi() -> None:
    assert RuntimeOperation.MOVE_JOINT_TRAJECTORY == 10
    assert RuntimeOperation.CANCEL_MOTION == 11
    assert RuntimeOperation.MOVE_POSE == 12
    assert RuntimeOperation.STOP_MOTION == 13
    assert RuntimeOperation.MOVE_LINEAR == 14
    assert RuntimeOperation.MOVE_CIRCULAR == 15
