from pathlib import Path


def test_dual_arm_examples_match_single_arm_numbering() -> None:
    root = Path(__file__).resolve().parents[1] / "arx_d_can" / "examples"
    single = {path.name for path in (root / "single_arm").glob("example_*.py")}
    dual = {path.name for path in (root / "dual_arm").glob("example_*.py")}

    assert dual == single
    assert len(dual) == 12
