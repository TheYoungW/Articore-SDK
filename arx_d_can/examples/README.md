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
  --velocity 50 \
  --max-acceleration 4.00

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
python -m arx_d_can.examples.control.example_07_cartesian_orientation_ptp \
  --speed 20
python -m arx_d_can.examples.control.example_07_cartesian_linear \
  --side left --duration 12
python -m arx_d_can.examples.control.example_07_cartesian_linear \
  --side right --duration 12
python -m arx_d_can.examples.control.example_07_cartesian_circular \
  --side left --duration 15
python -m arx_d_can.examples.control.example_07_cartesian_circular \
  --side right --duration 15
python -m arx_d_can.examples.control.example_08_gravity_compensation
python -m arx_d_can.examples.control.example_11_bimanual_follow \
  --mode pv --leader left --speed 30 --delta-deg 8
python -m arx_d_can.examples.control.example_12_tcp_offset \
  --side left --offset=-0.004,0,-0.128,0,0,0

# 受控真机演示：先到 J4=90°、其余0°，右手主导并用普通 MIT 往返8°
python -m arx_d_can.examples.control.example_11_bimanual_follow \
  --mode mit --leader right --speed 30 --delta-deg 8
```

`set_tcp_offset()` 的偏移由 C++ Runtime 保存到当前 Runtime 会话，并同时用于
`get_pose()`、PTP、Linear 和 Circular 的 FK/IK。它不是 Python 本地换算，也不会写入
电机 Flash。未设置时，有夹爪产品继续使用内置 `tool0`，无夹爪产品继续使用 `link7`；
`reset_tcp_offset()` 恢复这个产品默认值。为防止运动中坐标系突变，偏移只能在
Runtime 未连接或 READY 且整机确认失能时修改。

轨迹录制和回放也属于 `control`：

```bash
python -m arx_d_can.examples.control.example_09_record_gravity_trajectory \
  --output trajectories/dual.json --seconds 30 --hz 100

python -m arx_d_can.examples.control.example_10_replay_trajectory \
  --input trajectories/dual.json --mode pv

python -m arx_d_can.examples.control.example_10_replay_trajectory \
  --input trajectories/dual.json --mode mit \
  --mit-kp "190,190,70,125,10,22,28" \
  --mit-kd "4.55,4.5,2,2.9,0.7,0.89,0.84" \
  --mit-feedforward-torque "0,0,0,0,0,0,0"
```

回放只把完整双臂路点和时间戳提交一次。五次插值与 500 Hz 发送由 C++ Runtime
完成，Python 不再维护 100 Hz 实时回放循环。这里的关节轨迹不同于笛卡尔 PTP：
笛卡尔 PTP 只做终点 IK 后执行普通 PV；关节轨迹才使用原生五次规划和状态/取消接口。

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

PV 单点控制使用带单次 `velocity=0..100` 的 `set_joint_pv()`；默认单次速度为 50。
该百分比直接映射为 `0..2 rad/s`；`set_max_acceleration()` 使用
`0.01..8.00 rad/s²`，默认 `4.00 rad/s²`，使用 `0.01` 分辨率并由 Runtime 校验，
SDK 不取整。达妙 POS_VEL 独立的驱动速度上限仍为 3 rad/s。SDK 不公开 Raw PV，
也不生成 reference 或速度斜坡。MIT 仍可通过
`set_joint_mit()` 的显式速度或高级 Raw MIT 参数控制。产品限位、参数合法性、通信看门狗
及安全状态仍由 C++ Runtime 负责。

笛卡尔控制包含四个 PV 示例：`example_07_cartesian_ptp`、
`example_07_cartesian_orientation_ptp`、`example_07_cartesian_linear` 和
`example_07_cartesian_circular`。基础 PTP 通过一次原子
双臂调用提交左右镜像的 tool0 目标，使双臂到达 `J4=90°、其余关节=0°` 对应位姿；
Runtime 先完成两侧 IK，再一次安装普通 PV 的 14 关节目标。PTP 返回 `None`，没有 motion ID、状态或取消
接口，默认速度为 50。Linear 根据 `--side` 选择镜像路径，以原默认起点作为中心，
通过三个进入 Runtime FIFO 的直线任务画边长 7 cm 的左右镜像等边三角形。Circular 同样根据侧别选择 `YZ` 平面的镜像路径，
通过两个进入 Runtime FIFO 的半圆任务执行半径 8 cm 的完整圆并返回起点。基础 PTP、
Linear 和 Circular 均可用命令行参数覆盖默认位姿。
`example_07_cartesian_orientation_ptp` 在相同双臂基准姿态上，用普通双臂 PTP
依次演示 Pitch、Roll、Yaw 约 90° 的双向摆动。基准姿态的 `pitch=-90°` 是 RPY
奇异点，因此示例按真实旋转矩阵定义三个 base_link 旋转轴，而不是直接对奇异点处的
欧拉角做加减。Pitch 端点会同步改变位置以满足 J6 产品限位；每一步均通过真实位姿与
关节速度反馈确认到位，并在开始各轴演示前等待用户确认。
Runtime 将 Linear/Circular 作为复合 FIFO 任务：如果当前规划参考不在显式起点，先用
普通 PV PTP 接近，再由真实反馈按 5 mm / 0.035 rad 和稳定速度确认起点，最后执行声明的
直线或圆弧；全部阶段共用一个 motion ID。位姿单位为米和弧度。Linear 和 Circular
只在 Runtime 返回 `completed` 后报告到位，不会把
`running + progress=100%` 误判为完成。

夹爪 `direct` 模式会持续追踪目标，不执行接触保持、堵转判断或过载退让。使用时必须关注夹爪温升、机械过载和被夹物体安全；默认应保持 `--mode protected`。
