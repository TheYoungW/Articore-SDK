# Yunyi 双臂示例

所有示例都通过 `ArxDCanDualArm` 操作完整双臂产品，并按用途分为
`control`、`diagnostics` 和 `maintenance` 三组。
默认通过 DDS Domain 0 连接 `robot_id=yunyi-001` 的 RK3588 Runtime 服务。

先进入项目环境：

```bash
conda activate at
cd ~/Articore-SDK
```

## control：运动和控制

### 必须区分：关节点到点、IK 与有限笛卡尔轨迹

底层提供普通 PV 关节点到点、整机 IK 和三种有限笛卡尔轨迹：

| 用户方法 | 类型 | 完成与停止 | 对应示例 |
| --- | --- | --- | --- |
| `solve_ik()` + `set_joint_pv()` | 显式查看 IK 解，再提交普通关节 PV | 用户按关节反馈判断 | `example_07_cartesian_ptp` |
| `move_pose()` | 五次时间律 Pose-to-Pose 轨迹 | `state.motion_arrived` / `stop_motion()` | `example_11_cartesian_orientation_ptp` |
| `move_linear()` | 单段板端直线运动 | `state.motion_arrived` / `stop_motion()` | `example_09_cartesian_linear_trajectory` |
| `move_circular()` | 经过 start/via/end 的圆弧 | `state.motion_arrived` / `stop_motion()` | `example_10_cartesian_circular_trajectory` |

所有 `move_*` 都是非阻塞发送接口，成功时返回 `None`。Runtime 同时只执行一个
有限笛卡尔运动；应用自行轮询 `read_state().motion_arrived`，结合 `get_health()`
处理失败，并设置自己的等待超时。不要连续发送后依赖隐式队列。

```bash
python -m arx_d_can.examples.control.example_01_switch_control_mode --mode mit
python -m arx_d_can.examples.control.example_02_enable_disable

python -m arx_d_can.examples.control.example_03_send_position_pv \
  --left "0,0,0,90,0,0,0" \
  --right "0,0,0,90,0,0,0" \
  --velocity 50

python -m arx_d_can.examples.control.example_04_send_position_mit \
  --left "0,0,0,20,0,-20,0" \
  --right "0,0,0,20,0,-20,0" \
  --kp "40,40,35,30,25,20,15" \
  --kd "2,2,1.8,1.5,1.2,1.0,0.8"

# 遥操/高频控制接口：上层每次收到新目标时重复调用
# 安全警告：只使用小角度连续目标；请勿提交与当前姿态差异过大的目标
python -m arx_d_can.examples.control.example_17_send_position_mit_fast \
  --velocity 10

python -m arx_d_can.examples.control.example_05_set_gripper_openings \
  --left-gripper 1000 \
  --right-gripper 1000 \
  --gripper-level 5 \
  --mode protected

python -m arx_d_can.examples.control.example_06_return_zero
python -m arx_d_can.examples.control.example_07_cartesian_ptp
python -m arx_d_can.examples.control.example_11_cartesian_orientation_ptp \
  --speed 50
python -m arx_d_can.examples.control.example_09_cartesian_linear_trajectory \
  --side left --speed 50
python -m arx_d_can.examples.control.example_09_cartesian_linear_trajectory \
  --side right --speed 50
python -m arx_d_can.examples.control.example_10_cartesian_circular_trajectory \
  --side left --speed 50
python -m arx_d_can.examples.control.example_10_cartesian_circular_trajectory \
  --side right --speed 50
python -m arx_d_can.examples.control.example_12_gravity_compensation
python -m arx_d_can.examples.control.example_15_bimanual_follow \
  --mode pv --leader left --speed 30 --delta-deg 8
python -m arx_d_can.examples.control.example_16_tcp_offset \
  --side left --offset=-0.004,0,-0.128,0,0,0

# 受控真机演示：先到 J4=90°、其余0°，右手主导并用快速 MIT 往返8°
python -m arx_d_can.examples.control.example_15_bimanual_follow \
  --mode mit --leader right --delta-deg 8
```

`set_tcp_offset()` 的偏移由 C++ Runtime 保存到当前 Runtime 会话，并同时用于
`get_pose()`、`move_pose()`、Linear 和 Circular 的 FK/IK。它不是 Python 本地换算，也不会写入
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
`set_joint_pv()` 或 `set_joint_mit_fast()`，不会重采样或插值。`--velocity` 范围为
`1..100`，同时用于普通 PV 和快速 MIT。
底层独立提供的 Linear 和 Circular API 不受这个录制回放工具影响。

## diagnostics：读取和诊断

```bash
python -m arx_d_can.examples.diagnostics.example_01_read_state \
  --mode continuous --display-hz 10

python -m arx_d_can.examples.diagnostics.example_02_benchmark_read_rate \
  --seconds 15 --hz 500

python -m arx_d_can.examples.diagnostics.example_03_read_health
python -m arx_d_can.examples.diagnostics.example_04_read_pose
```

## maintenance：清错、恢复和调零

```bash
python -m arx_d_can.examples.maintenance.example_01_clear_faults
python -m arx_d_can.examples.maintenance.example_02_recover_to_zero
python -m arx_d_can.examples.maintenance.example_03_set_zero_current_position
```

`example_01_clear_faults` 使用 `connect(maintenance=True)`，只获取并维持控制
租约，不会在连接或清错成功后配置模式，全程不会自动使能或发送运动命令。若清错
失败，Runtime 返回的电机、操作和底层原因会原样抛出；不得用重启服务替代故障处理。
急停锁存仍会被 Runtime 拒绝，必须使用经过现场安全确认的恢复流程。

PV 单点控制使用 `velocity=1..100` 的 `set_joint_pv()`；默认 50。调用只提交最终
目标，新目标替换旧目标并保留 Runtime 的逐关节 V 爬坡状态。100% 速度上限为
`[180,180,180,225,225,225,225] deg/s`，100% 加速度上限为
`[450,450,900,900,900,900,900] deg/s²`；速度按 `s`、加速度按 `s²` 缩放。
`set_max_speed()` / `get_max_speed()` 与 `set_max_acceleration()` /
`get_max_acceleration()` 配置 100% 时共同作用于 14 关节的普通 PV 全局基础上限；0 表示恢复
Runtime 逐关节默认值。SDK 不对 `velocity` 重复缩放，也不静默裁剪 Runtime 拒绝的配置。
SDK 不公开 Raw/流式 PV，不生成 P 步进、速度包络或轨迹重采样。

标准 MIT 使用 `set_joint_mit()`，显式传入双臂 `q/dq/tau_ff` 和单臂 7 轴
`kp/kd`；7 轴增益按相同关节顺序应用到左右双臂，SDK 不选择控制增益。快速 MIT 使用
`set_joint_mit_fast(left=..., right=..., velocity=1..100)`，传角度和参考步进速度
百分比；固定增益、`dq=0`、`tau_ff=0` 由 C++ Runtime 负责，100 对应 5 rad/s。
`example_04` 会先读取当前反馈并执行最大角差检查，然后只提交一次完整双臂 MIT 帧；
默认拒绝任何关节相对反馈超过 `20°` 的总目标。示例的 J1..J7 默认 Kp 为
`[40,40,35,30,25,20,15]`，默认 Kd 为 `[2,2,1.8,1.5,1.2,1,0.8]`；也可通过
`--kp/--kd` 显式覆盖。通用 SDK 接口本身仍不选择控制增益。

末端位姿控制包含四个 PV 示例：`example_07_cartesian_ptp`、
`example_09_cartesian_linear_trajectory`、`example_10_cartesian_circular_trajectory` 和
`example_11_cartesian_orientation_ptp`。基础 Pose-to-Pose 示例先通过一次整机
`solve_ik()` 求左右镜像 tool0 目标，再调用一次 `set_joint_pv()`，使双臂到达
`J4=90°、其余关节=0°` 对应位姿；IK 阶段不使能也不运动。普通 PV 默认速度为
50。`move_pose()` 使用 Runtime 有限轨迹规划。
Linear 根据 `--side` 选择镜像路径，从默认起点沿 `base_link` 横向向外移动 15 cm
（原 10 cm 路径的 1.5 倍），只提交一次板端直线请求；Python 不生成或插值
Runtime 内部轨迹点。Circular 同样根据侧别选择 `YZ` 平面的镜像路径，
等待第一段到位后再提交返回半圆，执行半径 10 cm 的完整圆并返回起点。`move_pose()`、
Linear 和 Circular 均可用命令行参数覆盖默认位姿。
两个轨迹示例使用 `--speed 1..100` 调用 `set_speed_percent()`，不接收 duration。
Runtime 在提交时捕获速度百分比，并根据路径、IK 关节变化和内部限制自动定时。
内部规划 knot 间隔自适应为 4～50 ms，Runtime 在 knot 间线性重采样，仍以
500 Hz 下发内部 PV 轨迹命令。自动接近起点同样由 Runtime 管理。三种有限笛卡尔
轨迹都要求 PV 产品模式。
`example_11_cartesian_orientation_ptp` 在相同双臂基准姿态上，用 `move_pose()`
依次演示 Pitch、Roll、Yaw 约 90° 的双向摆动。基准姿态的 `pitch=-90°` 是 RPY
奇异点，因此示例按真实旋转矩阵定义三个 base_link 旋转轴，而不是直接对奇异点处的
欧拉角做加减。Pitch 端点会同步改变位置以满足 J6 产品限位；每一步均通过真实位姿与
关节速度反馈确认到位，并在开始各轴演示前等待用户确认。
如果当前规划参考不在 Linear/Circular 的显式起点，Runtime 会先用
普通 PV 关节点到点接近，再由真实反馈按 5 mm / 0.035 rad 和稳定速度确认起点，最后执行声明的
直线或圆弧。位姿单位为米和弧度。只有 `motion_arrived=True` 才表示 Runtime 已用
真实反馈确认到位；示例同时监控健康状态，并在超时时调用 `stop_motion()`。

夹爪 `direct` 模式会持续追踪目标，不执行接触保持、堵转判断或过载退让。使用时必须关注夹爪温升、机械过载和被夹物体安全；默认应保持 `--mode protected`。
