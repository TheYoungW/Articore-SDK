# Articore native safety runtime

This directory contains product policy for an Articore dual-arm robot. It depends on the generic
motor-drive-layer C ABI but does not add Yunyi or Articore concepts to motor-drive-layer.

The runtime owns one persistent worker thread. A complete PV or MIT command for both arm channels
is validated and sent through `ControllerGroup`; only a successful full-group send refreshes the
native `steady_clock` watchdog. The worker independently performs command timeout handling,
feedback and transport-health checks, safe-hold transmission, fault latching, linked disable, and
disable confirmation while Python is blocked or has stopped running.

PV safe hold preserves the last successful target with a dedicated low velocity limit. MIT safe
hold preserves position, zeros velocity and feedforward torque, and substitutes product safety
Kp/Kd. Grippers preserve the last successful position with low gains; overload during safe hold
causes one bounded retreat from the measured position. If no complete arm safety target exists,
the runtime disables and enters `FAULT` instead of sending an empty hold.

`runtime_abi.h` is the stable boundary used by `arx_d_can.sdk.native_safety`. The optional generic
motor-drive-layer transport-health callback is used when available; the wrapper remains compatible
with motor-drive-layer 0.5.9 and falls back to motor feedback health when that ABI function is not
present.

Build and run the native tests with:

```bash
cmake -S cpp_runtime -B build/runtime
cmake --build build/runtime
ctest --test-dir build/runtime --output-on-failure
```
