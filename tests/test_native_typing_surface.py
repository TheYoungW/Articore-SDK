from __future__ import annotations

import ast
from pathlib import Path


def _untyped_public_methods(path: Path, class_name: str) -> dict[str, list[str]]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            missing: dict[str, list[str]] = {}
            for item in node.body:
                if not isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    continue
                if item.name.startswith("_") and item.name not in {
                    "__init__",
                    "__enter__",
                    "__exit__",
                }:
                    continue
                method_missing = [
                    argument.arg
                    for argument in (
                        *item.args.posonlyargs,
                        *item.args.args,
                        *item.args.kwonlyargs,
                    )
                    if argument.arg not in {"self", "cls"}
                    and argument.annotation is None
                ]
                if item.args.vararg and item.args.vararg.annotation is None:
                    method_missing.append(f"*{item.args.vararg.arg}")
                if item.args.kwarg and item.args.kwarg.annotation is None:
                    method_missing.append(f"**{item.args.kwarg.arg}")
                if item.returns is None:
                    method_missing.append("return")
                if method_missing:
                    missing[item.name] = method_missing
            return missing
    raise AssertionError(f"missing class {class_name} in {path}")


def test_core_public_api_uses_inline_types() -> None:
    package = Path(__file__).resolve().parents[1] / "arx_d_can" / "_motor_abi"
    runtime = package / "core.py"

    assert (package / "py.typed").is_file()
    assert not list(package.rglob("*.pyi"))
    for class_name in (
        "Controller",
        "Motor",
        "ControllerGroup",
        "PreparedMitBatch",
        "PreparedPosVelBatch",
    ):
        assert _untyped_public_methods(runtime, class_name) == {}
