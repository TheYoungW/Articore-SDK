from __future__ import annotations

import math

import pytest

from arx_d_can.service_tools import calibrate_corina_ankle as calibration


CAPTURED_IMU_FRAME = bytes.fromhex(
    "fc 40 38 e9 7c 99 17 "
    "22 f2 06 bb a7 d9 f2 3a 3e 0c 39 bb da 24 db bf "
    "8c ab 06 c1 c6 6f 97 c0 38 2e d6 c2 76 46 94 44 "
    "e2 be 3a c4 73 fd 1e 42 80 e6 c5 47 73 fd 1e 42 "
    "2e e9 d4 26 00 00 00 00 fd"
)

CAPTURED_AHRS_FRAME = bytes.fromhex(
    "fc 41 30 ea 43 80 13 "
    "6b 4f 00 bb 72 b3 cf 3a 57 06 db ba 50 af 87 3f "
    "6d c2 37 be 9d 5a 59 40 22 b6 1d be bd 3d 51 3c "
    "a4 5e 02 3f 6c bc 58 3f b6 fc d4 26 00 00 00 00 fd"
)


def test_fdilink_parser_resynchronizes_and_decodes_captured_packets() -> None:
    parser = calibration.FdilinkParser()
    stream = b"\x00\x01noise" + CAPTURED_IMU_FRAME + CAPTURED_AHRS_FRAME
    frames = []
    for offset in range(0, len(stream), 13):
        frames.extend(parser.feed(stream[offset : offset + 13], host_monotonic_ns=123))

    assert parser.bad_frames == 0
    assert [frame.packet_id for frame in frames] == [0x40, 0x41]
    imu = calibration.decode_imu_frame(frames[0])
    ahrs = calibration.decode_ahrs_frame(frames[1])
    assert imu.sequence == 0xE9
    assert imu.device_timestamp_us > 0
    assert ahrs.sequence == 0xEA
    assert ahrs.device_timestamp_us > 0
    assert math.sqrt(sum(value * value for value in ahrs.quaternion)) == pytest.approx(1.0)


def test_fdilink_parser_rejects_bad_payload_crc() -> None:
    damaged = bytearray(CAPTURED_IMU_FRAME)
    damaged[20] ^= 0x01
    parser = calibration.FdilinkParser()
    assert parser.feed(bytes(damaged)) == []
    assert parser.bad_frames >= 1


def test_relative_quaternion_returns_expected_roll() -> None:
    half_angle = math.radians(10.0) / 2.0
    current = (math.cos(half_angle), math.sin(half_angle), 0.0, 0.0)
    relative, (roll, pitch, yaw) = calibration.relative_orientation((1.0, 0.0, 0.0, 0.0), current)
    assert relative == pytest.approx(current)
    assert math.degrees(roll) == pytest.approx(10.0)
    assert pitch == pytest.approx(0.0)
    assert yaw == pytest.approx(0.0)


def test_micro_plan_returns_to_zero_after_every_probe() -> None:
    points = calibration.micro_scan_points([1.0])
    assert points[0] == calibration.ScanPoint("zero", 0.0, 0.0)
    assert len(points) == 17
    assert all(
        points[index] == calibration.ScanPoint("return_zero", 0.0, 0.0)
        for index in range(2, len(points), 2)
    )


def test_grid_plan_is_serpentine() -> None:
    points = calibration.grid_scan_points(
        a_min=-1.0,
        a_max=1.0,
        a_step=1.0,
        b_min=-1.0,
        b_max=0.0,
        b_step=1.0,
    )
    assert [(point.a_relative_deg, point.b_relative_deg) for point in points[1:4]] == [
        (-1.0, -1.0),
        (0.0, -1.0),
        (1.0, -1.0),
    ]
    assert [(point.a_relative_deg, point.b_relative_deg) for point in points[4:7]] == [
        (1.0, 0.0),
        (0.0, 0.0),
        (-1.0, 0.0),
    ]


def test_runtime_config_selects_one_leg_and_removes_test_pair_virtual_limits() -> None:
    args = calibration.parser().parse_args(["--side", "right"])
    config = calibration.build_runtime_config(args)
    by_name = {joint.name: joint for joint in config.arm_joints}
    assert list(by_name) == [f"right_leg_joint{index}" for index in range(1, 7)]
    assert by_name["right_leg_joint1"].lower_limit is not None
    assert by_name["right_leg_joint4"].upper_limit is not None
    assert by_name["right_leg_joint5"].lower_limit is None
    assert by_name["right_leg_joint5"].upper_limit is None
    assert by_name["right_leg_joint6"].lower_limit is None
    assert by_name["right_leg_joint6"].upper_limit is None
