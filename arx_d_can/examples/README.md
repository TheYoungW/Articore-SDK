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
  --max-speed 50

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
python -m arx_d_can.examples.control.example_07_cartesian_ptp
python -m arx_d_can.examples.control.example_07_cartesian_linear \
  --side left --speed 20
python -m arx_d_can.examples.control.example_07_cartesian_linear \
  --side right --speed 20
python -m arx_d_can.examples.control.example_07_cartesian_circular \
  --side left --speed 20
python -m arx_d_can.examples.control.example_07_cartesian_circular \
  --side right --speed 20
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

PV 单点控制只使用 `set_max_speed(0..100)` 和不带速度参数的 `set_joint_pv()`；0～100 线性对应 0～2 rad/s reference slew，产品默认值为 50，对应 1 rad/s 和 500 Hz 下每周期 0.002 rad，100 对应每周期 0.004 rad。达妙 POS_VEL 的 `V` 始终固定为 3 rad/s，不随百分比缩放。SDK 不再公开 Raw PV。MIT 仍可通过 `set_joint_mit()` 的显式速度或高级 Raw MIT 参数控制。产品限位、参数合法性、通信看门狗及安全状态仍由 C++ Runtime 负责。

笛卡尔控制拆分为三个 PV 示例：`example_07_cartesian_ptp`、
`example_07_cartesian_linear` 和 `example_07_cartesian_circular`。PTP 通过一次原子
双臂调用提交左右镜像的 tool0 目标，使双臂到达 `J4=90°、其余关节=0°` 对应位姿；
Runtime 先完成两侧 IK，再一次安装普通 PV 的 14 关节目标。PTP 返回 `None`，没有 motion ID、状态或取消
接口，默认速度为 50。Linear 根据 `--side` 选择镜像路径：左臂沿 `+Y`、右臂
沿 `-Y` 横向向外直线移动 10 cm。Circular 同样根据侧别选择 `YZ` 平面的镜像路径，
执行半径约 6 cm、向手臂外侧鼓出的半圆。三个示例均可用命令行参数覆盖默认位姿。
Runtime 将 Linear/Circular 作为复合 FIFO 任务：如果当前规划参考不在显式起点，先用
普通 PV PTP 接近，再由真实反馈按 5 mm / 0.035 rad 和稳定速度确认起点，最后执行声明的
直线或圆弧；全部阶段共用一个 motion ID。位姿单位为米和弧度。Linear 和 Circular
只在 Runtime 返回 `completed` 后报告到位，不会把
`running + progress=100%` 误判为完成。

夹爪 `direct` 模式会持续追踪目标，不执行接触保持、堵转判断或过载退让。使用时必须关注夹爪温升、机械过载和被夹物体安全；默认应保持 `--mode protected`。
