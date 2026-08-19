# Yunyi 双臂示例

所有示例都通过 `ArxDCanDualArm` 操作完整双臂产品。

```bash
conda activate at
cd ~/Articore-SDK

python -m arx_d_can.examples.dual_arm.example_02_switch_control_mode --mode mit
python -m arx_d_can.examples.dual_arm.example_03_enable_disable
python -m arx_d_can.examples.dual_arm.example_04_read_state --mode continuous --display-hz 10
python -m arx_d_can.examples.dual_arm.example_05_clear_faults

python -m arx_d_can.examples.dual_arm.example_06_send_position_pv \
  --left "0,0,0,90,0,0,0" \
  --right "0,0,0,90,0,0,0" \
  --velocity 30

python -m arx_d_can.examples.dual_arm.example_07_send_position_mit \
  --left "0,0,0,90,0,0,0" \
  --right "0,0,0,90,0,0,0" \
  --velocity 30

python -m arx_d_can.examples.dual_arm.example_08_set_gripper_openings \
  --left-gripper 1000 \
  --right-gripper 1000 \
  --gripper-level 3

python -m arx_d_can.examples.dual_arm.example_09_benchmark_read_rate \
  --seconds 15 --hz 500 --cached

python -m arx_d_can.examples.dual_arm.example_12_diagnose_status
python -m arx_d_can.examples.dual_arm.example_13_set_zero_current_position
python -m arx_d_can.examples.dual_arm.example_15_gravity_compensation
```

轨迹录制和回放：

```bash
python -m arx_d_can.examples.dual_arm.example_16_record_gravity_trajectory \
  --output trajectories/dual.json --seconds 30 --hz 100

python -m arx_d_can.examples.dual_arm.example_17_replay_trajectory \
  --input trajectories/dual.json --mode pv --interpolation quintic
```

Python 不维护电机参数和产品限位；参数检查、控制周期、目标限步、通信看门狗及安全状态全部由 C++ Runtime 负责。
