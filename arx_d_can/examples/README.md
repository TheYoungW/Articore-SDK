# Yunyi 双臂示例

所有示例都通过 `ArxDCanDualArm` 操作完整双臂产品，并按用途分为
`control`、`diagnostics` 和 `maintenance` 三组。

先进入项目环境：

```bash
conda activate at
cd ~/Articore-SDK
```

## control：运动和控制

### 必须区分：关节点到点、一次终点 IK 与路径轨迹

底层提供普通 PV 关节点到点、整机 IK、兼容 Pose 快捷命令和三个原生轨迹提交接口：

| 用户方法 | Runtime C ABI | 类型 | motion ID / 状态 / 取消 | 对应示例 |
| --- | --- | --- | --- | --- |
| `solve_ik()` + `set_joint_pv()` | `articore_runtime_solve_ik()` + `articore_runtime_set_joint_pv()` | 推荐 Pose-to-Pose：只求一次整机 IK，再提交普通 PV | 无 | `example_07_cartesian_ptp` |
| `set_pose()` | `articore_runtime_set_pose()` | 兼容快捷入口：一次终点 IK 后按当前普通 PV 或 MIT 模式执行 | 无 | — |
| `move_joint_trajectory()` | `articore_runtime_move_joint_trajectory()` | 原生双臂关节轨迹 | 有 | `example_08_joint_trajectory` |
| `move_linear_trajectory()` | `articore_runtime_move_linear_trajectory()` / `articore_runtime_move_linear_path_trajectory()` | 原生直线或默认 10 mm 圆角融合路径 | 有 | `example_09_cartesian_linear_trajectory` |
| `move_circular_trajectory()` | `articore_runtime_move_circular_trajectory()` | 原生圆弧轨迹 | 有 | `example_10_cartesian_circular_trajectory` |

`example_11_cartesian_orientation_ptp` 只是 `set_pose()` 的姿态演示，不是第四种
轨迹接口。`solve_ik()` 只返回双臂关节解，不使能、不运动、不改变 FIFO；
`example_07_cartesian_ptp` 随后只调用一次 `set_joint_pv()`。Linear 和 Circular 才进入
原生 FIFO，并通过 `get_motion_status()` / `cancel_motion()` 管理。

```bash
python -m arx_d_can.examples.control.example_01_switch_control_mode --mode mit
python -m arx_d_can.examples.control.example_02_enable_disable

python -m arx_d_can.examples.control.example_03_send_position_pv \
  --velocity 50 \
  --max-acceleration 6.00

python -m arx_d_can.examples.control.example_04_send_position_mit \
  --left "0,0,0,90,0,0,0" \
  --right "0,0,0,90,0,0,0" \
  --velocity 10

python -m arx_d_can.examples.control.example_05_set_gripper_openings \
  --left-gripper 1000 \
  --right-gripper 1000 \
  --gripper-level 5 \
  --mode protected

python -m arx_d_can.examples.control.example_06_return_zero --velocity 50
python -m arx_d_can.examples.control.example_07_cartesian_ptp
python -m arx_d_can.examples.control.example_08_joint_trajectory \
  --mode pv --duration 5
python -m arx_d_can.examples.control.example_11_cartesian_orientation_ptp \
  --speed 50
python -m arx_d_can.examples.control.example_09_cartesian_linear_trajectory \
  --side left --duration 10
python -m arx_d_can.examples.control.example_09_cartesian_linear_trajectory \
  --side right --duration 3
python -m arx_d_can.examples.control.example_10_cartesian_circular_trajectory \
  --side left --duration 10
python -m arx_d_can.examples.control.example_10_cartesian_circular_trajectory \
  --side right --duration 10
python -m arx_d_can.examples.control.example_12_gravity_compensation
python -m arx_d_can.examples.control.example_15_bimanual_follow \
  --mode pv --leader left --speed 30 --delta-deg 8
python -m arx_d_can.examples.control.example_16_tcp_offset \
  --side left --offset=-0.004,0,-0.128,0,0,0

# 受控真机演示：先到 J4=90°、其余0°，右手主导并用普通 MIT 往返8°
python -m arx_d_can.examples.control.example_15_bimanual_follow \
  --mode mit --leader right --speed 30 --delta-deg 8
```

`set_tcp_offset()` 的偏移由 C++ Runtime 保存到当前 Runtime 会话，并同时用于
`get_pose()`、`set_pose()`、Linear 和 Circular 的 FK/IK。它不是 Python 本地换算，也不会写入
电机 Flash。未设置时，有夹爪产品继续使用内置 `tool0`，无夹爪产品继续使用 `link7`；
`reset_tcp_offset()` 恢复这个产品默认值。为防止运动中坐标系突变，偏移只能在
Runtime 未连接或 READY 且整机确认失能时修改。

轨迹录制和回放也属于 `control`：

```bash
python -m arx_d_can.examples.control.example_13_record_gravity_trajectory \
  --output trajectories/dual.json --seconds 30 --hz 500

python -m arx_d_can.examples.control.example_14_replay_trajectory \
  --input trajectories/dual.json --mode pv --velocity 50

python -m arx_d_can.examples.control.example_14_replay_trajectory \
  --input trajectories/dual.json --mode mit --velocity 50
```

录制频率范围为 `(0, 500] Hz`。Python 回放按文件中的原始时间戳逐点调用普通
`set_joint_pv()` 或 `set_joint_mit()`，不会重采样、插值或调用原生
`move_joint_trajectory()`。`--velocity` 使用对应普通 PV/MIT 命令的 `0..100` 速度档位。
底层独立提供的关节轨迹、Linear 和 Circular API 不受这个录制回放工具影响。

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
`0.01..8.00 rad/s²`，默认 `6.00 rad/s²`，使用 `0.01` 分辨率并由 Runtime 校验，
SDK 不取整。达妙 POS_VEL 独立的驱动速度上限仍为 3 rad/s。SDK 不公开 Raw/流式 PV，
`set_joint_pv()` 是用户普通步进/点到点接口；实时 PV 只由 Runtime 的有限轨迹内部使用，
SDK 不生成 reference 或速度斜坡。MIT 仍可通过
`set_joint_mit()` 的显式速度或高级 Raw MIT 参数控制。产品限位、参数合法性、通信看门狗
及安全状态仍由 C++ Runtime 负责。

末端位姿控制包含四个 PV 示例：`example_07_cartesian_ptp`、
`example_09_cartesian_linear_trajectory`、`example_10_cartesian_circular_trajectory` 和
`example_11_cartesian_orientation_ptp`。基础 Pose-to-Pose 示例先通过一次整机
`solve_ik()` 求左右镜像 tool0 目标，再调用一次 `set_joint_pv()`，使双臂到达
`J4=90°、其余关节=0°` 对应位姿；IK 阶段不使能也不运动。普通 PV 默认速度为
50。兼容 `set_pose()` 快捷入口仍返回 `None`，没有 motion ID、状态或取消接口。
Linear 根据 `--side` 选择镜像路径，以原默认起点作为中心，
通过一个 Motion ID 和默认 10 mm 圆角融合路径画边长 14 cm 的左右镜像等边三角形。Circular 同样根据侧别选择 `YZ` 平面的镜像路径，
通过两个进入 Runtime FIFO 的半圆任务执行半径 10 cm 的完整圆并返回起点。基础 `set_pose()`、
Linear 和 Circular 均可用命令行参数覆盖默认位姿。
Linear 的 `--duration` 表示每条原始边的参考时间：`3` 秒的三角形总参考时间约
9 秒，通常共 900 段、901 点。Runtime 对两个内部运行角点默认加入 10 mm 圆角，
整条路径只使用一条全局五次时间律和一个 Motion ID；起点/最终点仍正常减速到停。
Runtime 先保证笛卡尔几何精度，再生成固定 10 ms 的内部实时 PV 参考。
真实到位可能晚于参考时间。自动接近起点同样使用普通 PV。Linear、Circular 要求
PV 产品模式；`set_pose()` 按 Runtime 当前普通 PV 或 MIT 模式执行。
`example_11_cartesian_orientation_ptp` 在相同双臂基准姿态上，用 `set_pose()`
依次演示 Pitch、Roll、Yaw 约 90° 的双向摆动。基准姿态的 `pitch=-90°` 是 RPY
奇异点，因此示例按真实旋转矩阵定义三个 base_link 旋转轴，而不是直接对奇异点处的
欧拉角做加减。Pitch 端点会同步改变位置以满足 J6 产品限位；每一步均通过真实位姿与
关节速度反馈确认到位，并在开始各轴演示前等待用户确认。
Runtime 将 Linear/Circular 作为复合 FIFO 任务：如果当前规划参考不在显式起点，先用
普通 PV 关节点到点接近，再由真实反馈按 5 mm / 0.035 rad 和稳定速度确认起点，最后执行声明的
直线或圆弧；全部阶段共用一个 motion ID。位姿单位为米和弧度。Linear 和 Circular
只在 Runtime 返回 `completed` 后报告到位，不会把
`running + progress=100%` 误判为完成。

夹爪 `direct` 模式会持续追踪目标，不执行接触保持、堵转判断或过载退让。使用时必须关注夹爪温升、机械过载和被夹物体安全；默认应保持 `--mode protected`。
