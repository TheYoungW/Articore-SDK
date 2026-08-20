# Yunyi 双臂示例

所有示例都通过 `ArxDCanDualArm` 操作完整双臂产品。

```bash
conda activate at
cd ~/Articore-SDK

python -m arx_d_can.examples.example_02_switch_control_mode --mode mit
python -m arx_d_can.examples.example_03_enable_disable
python -m arx_d_can.examples.example_04_read_state --mode continuous --display-hz 10
python -m arx_d_can.examples.example_05_clear_faults

python -m arx_d_can.examples.example_06_send_position_pv \
  --left "0,0,0,90,0,0,0" \
  --right "0,0,0,90,0,0,0" \
  --velocity 30

python -m arx_d_can.examples.example_07_send_position_mit \
  --left "0,0,0,90,0,0,0" \
  --right "0,0,0,90,0,0,0" \
  --velocity 10

python -m arx_d_can.examples.example_08_set_gripper_openings \
  --left-gripper 1000 \
  --right-gripper 1000 \
  --gripper-level 3

python -m arx_d_can.examples.example_09_benchmark_read_rate \
  --seconds 15 --hz 500 --cached

python -m arx_d_can.examples.example_12_diagnose_status
python -m arx_d_can.examples.example_13_set_zero_current_position
python -m arx_d_can.examples.example_15_gravity_compensation
```

轨迹录制和回放：

```bash
python -m arx_d_can.examples.example_16_record_gravity_trajectory \
  --output trajectories/dual.json --seconds 30 --hz 100

python -m arx_d_can.examples.example_17_replay_trajectory \
  --input trajectories/dual.json --mode pv --interpolation quintic

python -m arx_d_can.examples.example_17_replay_trajectory \
  --input trajectories/dual.json --mode mit --interpolation quintic \
  --mit-target-velocity "0,0,0,0,0,0,0" \
  --mit-kp "190,190,70,125,10,22,28" \
  --mit-kd "4.55,4.5,2,2.9,0.7,0.89,0.84" \
  --mit-feedforward-torque "0,0,0,0,0,0,0"
```

MIT 单点控制默认使用普通 `set_joint_mit()`，速度为显式填写的 0～100 档位。轨迹回放属于高级接口，会显式提交完整 raw MIT 帧。产品限位、参数合法性、通信看门狗及安全状态仍由 C++ Runtime 负责。
