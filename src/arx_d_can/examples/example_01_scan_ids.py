#!/usr/bin/env python3
"""Example 01: scan Damiao motor IDs without enabling the arm."""
from __future__ import annotations

import argparse

from arx_d_can import ArxDCanArm


def main(args: argparse.Namespace) -> None:
    arm = ArxDCanArm(port=args.port, baud=args.baud)
    ids = arm.scan_ids(
        start_id=args.start_id,
        end_id=args.end_id,
        model=args.model,
        timeout_ms=args.timeout_ms,
    )
    print("found:", " ".join(f"0x{motor_id:02X}" for motor_id in ids) or "none")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Scan Damiao motor IDs without enabling the arm.")
    parser.add_argument("--start-id", type=int, default=1)
    parser.add_argument("--end-id", type=int, default=16)
    parser.add_argument("--model", default="4340P")
    parser.add_argument("--timeout-ms", type=int, default=30)
    parser.add_argument("--port", default="/dev/ttyACM0", help="USB2CAN serial port")
    parser.add_argument("--baud", type=int, default=1_000_000, help="USB2CAN serial baudrate")
    main(parser.parse_args())
