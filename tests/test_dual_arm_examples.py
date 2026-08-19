from pathlib import Path


def test_product_examples_are_flat_and_exclude_removed_runtime_demo() -> None:
    root = Path(__file__).resolve().parents[1] / "arx_d_can" / "examples"
    examples = sorted(path.name for path in root.glob("example_*.py"))

    assert not (root / "dual_arm").exists()
    assert examples == [
        "example_02_switch_control_mode.py",
        "example_03_enable_disable.py",
        "example_04_read_state.py",
        "example_05_clear_faults.py",
        "example_06_send_position_pv.py",
        "example_07_send_position_mit.py",
        "example_08_set_gripper_openings.py",
        "example_09_benchmark_read_rate.py",
        "example_11_return_zero.py",
        "example_12_diagnose_status.py",
        "example_13_set_zero_current_position.py",
        "example_15_gravity_compensation.py",
        "example_16_record_gravity_trajectory.py",
        "example_17_replay_trajectory.py",
    ]
