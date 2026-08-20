# Yunyi 双臂示例

所有示例都通过 `ArxDCanDualArm` 操作完整双臂产品，并按用途分为
`control`、`diagnostics` 和 `maintenance` 三组。

先进入项目环境：

```bash
conda activate at
cd ~/Articore-SDK
```

## control：运动和控制

```bash
python -m arx_d_can.examples.control.example_01_switch_control_mode --mode mit
python -m arx_d_can.examples.control.example_02_enable_disable

python -m arx_d_can.examples.control.example_03_send_position_pv \
  --left "0,0,0,90,0,0,0" \
  --right "0,0,0,90,0,0,0" \
  --velocity 30

python -m arx_d_can.examples.control.example_04_send_position_mit \
  --left "0,0,0,90,0,0,0" \
  --right "0,0,0,90,0,0,0" \
  --velocity 10

python -m arx_d_can.examples.control.example_05_set_gripper_openings \
  --left-gripper 1000 \
  --right-gripper 1000 \
  --gripper-level 5 \
  --mode protected

python -m arx_d_can.examples.control.example_06_return_zero --velocity 20
python -m arx_d_can.examples.control.example_07_cartesian_motion \
  --side left --motion linear \
  --target "0.35,0.20,0.30,0,0,0" --speed 20
python -m arx_d_can.examples.control.example_08_gravity_compensation
```

轨迹录制和回放也属于 `control`：

```bash
python -m arx_d_can.examples.control.example_09_record_gravity_trajectory \
  --output trajectories/dual.json --seconds 30 --hz 100

python -m arx_d_can.examples.control.example_10_replay_trajectory \
  --input trajectories/dual.json --mode pv --interpolation quintic

python -m arx_d_can.examples.control.example_10_replay_trajectory \
  --input trajectories/dual.json --mode mit --interpolation quintic \
  --mit-target-velocity "0,0,0,0,0,0,0" \
  --mit-kp "190,190,70,125,10,22,28" \
  --mit-kd "4.55,4.5,2,2.9,0.7,0.89,0.84" \
  --mit-feedforward-torque "0,0,0,0,0,0,0"
```

## diagnostics：读取和诊断

```bash
python -m arx_d_can.examples.diagnostics.example_01_read_state \
  --mode continuous --display-hz 10

python -m arx_d_can.examples.diagnostics.example_02_benchmark_read_rate \
  --seconds 15 --hz 500 --cached

python -m arx_d_can.examples.diagnostics.example_03_read_health
python -m arx_d_can.examples.diagnostics.example_04_read_pose
```

## maintenance：清错、恢复和调零

```bash
python -m arx_d_can.examples.maintenance.example_01_clear_faults
python -m arx_d_can.examples.maintenance.example_02_recover_to_zero
python -m arx_d_can.examples.maintenance.example_03_set_zero_current_position
```

MIT 单点控制默认使用普通 `set_joint_mit()`，速度为显式填写的 0～100 档位。轨迹回放属于高级接口，会显式提交完整 raw MIT 帧。产品限位、参数合法性、通信看门狗及安全状态仍由 C++ Runtime 负责。

控制示例 07 只用于 PV 模式，一次只控制 `left` 或 `right`。点到点使用 `--motion ptp`，直线使用 `linear`；圆弧使用 `circular`，并额外提供 `--start` 和 `--via`。位姿单位为米和弧度。示例只在 Runtime 返回 `completed` 后报告到位，不会把 `running + progress=100%` 误判为完成。

夹爪 `direct` 模式会持续追踪目标，不执行接触保持、堵转判断或过载退让。使用时必须关注夹爪温升、机械过载和被夹物体安全；默认应保持 `--mode protected`。
