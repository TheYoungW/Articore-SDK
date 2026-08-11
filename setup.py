from __future__ import annotations

import os
import sys
from pathlib import Path

from setuptools import Extension, setup


ROOT = Path(__file__).resolve().parent
compile_args = ["/std:c++17", "/EHsc"] if os.name == "nt" else ["-std=c++17"]
if sys.platform != "win32":
    compile_args.extend(["-fvisibility=hidden", "-pthread"])

setup(
    ext_modules=[
        Extension(
            "arx_d_can._articore_runtime_native",
            sources=[
                str(ROOT / "cpp_runtime" / "src" / "runtime.cpp"),
                str(ROOT / "cpp_runtime" / "src" / "runtime_abi.cpp"),
            ],
            include_dirs=[
                str(ROOT / "cpp_runtime" / "include"),
                str(ROOT / "cpp_runtime" / "src"),
            ],
            language="c++",
            extra_compile_args=compile_args,
            extra_link_args=[] if os.name == "nt" else ["-pthread"],
        )
    ]
)
