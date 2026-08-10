import pytest

from arx_d_can.sdk.gripper_force_control import (
    GripperControlState,
    GripperForceControlConfig,
    GripperForceController,
)


def controller(**overrides) -> GripperForceController:
    values = {
        "close_speed": 5.0,
        "contact_torque": 0.8,
        "overload_torque": 1.5,
        "motion_window_s": 0.1,
        "stall_movement": 0.01,
        "min_position_error": 0.05,
        "contact_hold_s": 0.1,
        "overload_hold_s": 0.05,
        "hold_offset": 0.08,
        "retreat_distance": 0.15,
        "max_step_interval_s": 0.1,
        "overload_retreat_interval_s": 0.1,
        "hold_kp": 2.0,
        "hold_kd": 0.5,
    }
    values.update(overrides)
    return GripperForceController(
        GripperForceControlConfig(**values),
        open_value=2.64,
        closed_value=0.0,
        normal_kp=4.0,
        normal_kd=0.5,
    )


def test_stalled_closing_switches_from_default_kp_to_safe_hold() -> None:
    gripper = controller()

    first = gripper.update(
        requested_position=0.0,
        actual_position=2.0,
        actual_torque=0.9,
        now=0.0,
    )
    gripper.update(
        requested_position=0.0,
        actual_position=2.0,
        actual_torque=0.9,
        now=0.1,
    )
    holding = gripper.update(
        requested_position=0.0,
        actual_position=2.0,
        actual_torque=0.9,
        now=0.2,
    )

    assert first.kp == pytest.approx(4.0)
    assert holding.state is GripperControlState.HOLDING
    assert holding.kp == pytest.approx(2.0)
    assert holding.position == pytest.approx(1.92)


def test_sustained_overload_retreats_toward_open_position() -> None:
    gripper = controller()

    gripper.update(
        requested_position=0.0,
        actual_position=2.0,
        actual_torque=2.0,
        now=0.0,
    )
    overloaded = gripper.update(
        requested_position=0.0,
        actual_position=2.0,
        actual_torque=2.0,
        now=0.05,
    )

    assert overloaded.state is GripperControlState.OVERLOAD
    assert overloaded.kp == pytest.approx(2.0)
    assert overloaded.position == pytest.approx(2.15)
