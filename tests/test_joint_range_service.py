from __future__ import annotations

import math

import pytest

from arx_d_can import default_config
from arx_d_can.service_tools import joint_range_test as service


def test_parser_defaults_to_corina_right_leg_joints_1_through_4() -> None:
    args = service.build_parser().parse_args([])

    assert args.arm_model is None
    assert args.joints == tuple(f"right_leg_joint{index}" for index in range(1, 5))
    assert args.range_percent == 95.0
    assert args.duration == 6.0
    assert args.hz == 200.0


def test_select_joint_config_preserves_requested_order_and_directions() -> None:
    config = default_config(model="corina_v2")

    selected = service.select_joint_config(
        config,
        ("right_leg_joint4", "right_leg_joint1"),
    )

    assert selected.joint_names == ("right_leg_joint4", "right_leg_joint1")
    assert [joint.direction for joint in selected.arm_joints] == [-1.0, -1.0]
    assert selected.joint_transform_path is None
    assert selected.gripper is None

    arm = service.ArxDCanArm(config=selected)
    assert arm.joint_names == ("right_leg_joint4", "right_leg_joint1")


def test_sweep_targets_reach_95_percent_of_each_urdf_limit() -> None:
    config = default_config(model="corina_v2")
    joint = next(
        joint for joint in config.arm_joints if joint.name == "right_leg_joint2"
    )

    lower, upper = service.calculate_sweep_targets(
        joint,
        math.radians(-10.0),
        range_fraction=0.95,
    )

    assert math.degrees(lower) == pytest.approx(-10.83)
    assert math.degrees(upper) == pytest.approx(38.095)


def test_sweep_rejects_start_outside_urdf_limits() -> None:
    config = default_config(model="corina_v2")
    joint = next(
        joint for joint in config.arm_joints if joint.name == "right_leg_joint4"
    )

    with pytest.raises(ValueError, match="starts outside its URDF range"):
        service.calculate_sweep_targets(
            joint,
            math.radians(-1.0),
            range_fraction=0.95,
        )
