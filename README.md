# Articore-SDK

Yunyi V1.0 双臂 Python SDK。这个仓库只支持完整双臂产品；单臂支持将放在独立项目中。

电机通信、双通道 Runtime、产品配置、限位、安全状态机、夹爪、运动学、动力学和重力补偿均由 `motor-drive-layer` 的 C++ Runtime 负责。Python 只保留双臂业务接口和 ctypes 绑定。

## 安装

```bash
conda activate at
pip install -e .
```

当前版本严格依赖 `motor-drive-layer==0.23.0`，并严格要求 Runtime ABI 11.4
(`0x000B0004`)；ABI 不一致时直接拒绝加载，不执行旧 ABI 兼容。x86_64 与
AArch64 由 pip 自动选择对应 wheel。Runtime 只通过三参数
`articore_runtime_create_yunyi(mode, with_grippers, &runtime)` 创建。PV reference、
500 Hz 安全循环、逐关节到位收敛及底层诊断均由 C++ Runtime 内部管理；SDK 只传递
参数和转换错误。URDF 继续随 SDK 分发，用于展示、仿真和外部工具；控制参数不从
Python YAML 读取。

## 最小用法

```python
from arx_d_can import ArxDCanDualArm

robot = ArxDCanDualArm(control_mode="pv", with_grippers=True)
try:
    robot.connect()
    robot.enable()

    robot.set_joint_pv(
        left=[0.0] * 7,
        right=[0.0] * 7,
        velocity=50,
    )
    robot.set_grippers(left=1000, right=1000, gripper_level=5)

    state = robot.read_state()
    print(state.left.positions)
    print(state.right.positions)
    print(robot.get_health())
    print(robot.get_fps())
finally:
    robot.disconnect()
```

创建对象只有两个业务参数：

- `control_mode="mit" | "pv"`
- `with_grippers=True | False`

没有夹爪时，状态中的左右夹爪都是 `None`；关节控制和关节状态仍然只包含左右各 7 个机械臂关节。

用户生命周期只使用 `connect()` 和 `disconnect()`。`disconnect()` 是终止型安全关闭，会由
C++ Runtime 完成整机失能确认、停止 worker、关闭双 CAN 并释放产品资源；重复调用安全。
Python 不公开单独的 `close()` 或 `free()`。

## 控制接口

- 普通 PV 位置：调用
  `set_joint_pv(left=..., right=..., velocity=1..100)` 直接提交最新最终目标；默认值为
  50。新目标替换旧目标并保留各关节当前 V 爬坡状态。100% 速度上限为
  `[180,180,180,225,225,225,225] deg/s`，100% 加速度上限为
  `[450,450,900,900,900,900,900] deg/s²`；速度按比例 `s` 缩放，加速度按
  `s²` 缩放。位置累积、V 包络、反馈量化补偿和最终端点精确到达全部由 Runtime
  负责。SDK 不生成 P 步进、速度斜坡或轨迹重采样，也不公开 Raw PV 直发。
  Joint/Linear/Circular 的速度、加速度和 jerk 完全由 Runtime 按用户给定时间内部规划，
  不读取普通 PV 设置，也不向用户暴露轨迹加速度。Linear/Circular 要求 PV 产品模式；
  `set_pose()` 跟随当前普通 PV 或 MIT 模式。
- 普通 MIT 位置使用 `set_joint_mit(left=..., right=...)`，只接收角度并采用
  latest-target-wins；Runtime 以 500 Hz 持续下发，固定 `dq=0`、`tau_ff=0`。
  遥操和高频控制使用 `set_joint_mit_fast_follow(left=..., right=...)`，同样只接收
  角度。Raw MIT 保留给明确需要
  自定义 Kp、Kd、目标速度和前馈力矩的高级控制。

两个公开 MIT 模式的固定增益由 Runtime 管理：

| 关节 | 普通 Kp | 普通 Kd | 快速跟随 Kp | 快速跟随 Kd |
| --- | ---: | ---: | ---: | ---: |
| J1 | 15 | 0.8 | 190 | 4.55 |
| J2 | 15 | 0.8 | 190 | 4.50 |
| J3 | 12 | 0.7 | 100 | 2.50 |
| J4 | 12 | 0.7 | 100 | 2.50 |
| J5 | 8 | 0.5 | 70 | 0.70 |
| J6 | 7 | 0.5 | 60 | 0.60 |
| J7 | 6 | 0.4 | 50 | 0.50 |

旧代码迁移：

```python
# 旧接口（velocity 已删除）
robot.set_joint_mit(left=left, right=right, velocity=100)

# 普通 MIT
robot.set_joint_mit(left=left, right=right)

# 遥操/高频跟随
robot.set_joint_mit_fast_follow(left=left, right=right)
```
- 电机电源：`enable()` / `disable()` 操作整机；传入
  `motors=["l-joint4", "r-joint4"]` 时由 C++ Runtime 执行一次原子批量事务。部分使能状态下
  仍提交完整 14 轴目标，底层自动跳过主动失能的电机。
- 原始帧：产品 SDK 只保留 `submit_raw_mit()`。用户普通 PV 必须走
  `set_joint_pv()` 最终目标路径；内部实时 PV 只能由有限轨迹触发，不公开
  `submit_raw_pv()` 或流式 PV 接口。
- 夹爪：`set_grippers(left=0..1000, right=0..1000, gripper_level=0..10, mode="protected" | "direct")`。默认 `protected`；`direct` 会持续追踪目标且不执行夹爪接触保持、堵转判断和过载退让。直驱时应关注夹爪温升、机械过载和被夹物体安全。
- 状态：`read_state()`、`read_cached_state()`。
  `state.left.arm.enabled` 和 `state.right.arm.enabled` 返回逐关节
  `tuple[bool | None, ...]`；夹爪的 `enabled` 使用相同语义。数据直接来自底层新鲜反馈缓存，
  `None` 表示缺失、过期或无法确认。
- 健康：`get_health()`、`get_fps()`。`get_health().motor_feedback` 按产品顺序返回
  每台电机的 role、CAN ID、反馈年龄、状态码和组合 issue bits；
  `feedback_issue_scope` 区分单电机、多电机、左通道、右通道和双通道异常。
- 产品关节限位：`get_joint_limits()` 无需连接或使能，也不会发送 CAN 请求。返回字典固定按
  `l-joint1..7`、`r-joint1..7` 排列，值包含 `min_angle_rad`、
  `max_angle_rad` 和 `max_velocity_rad_s`。这些值直接来自 C++ Runtime 实际用于
  PV、轨迹和笛卡尔校验的同一份产品配置，不从 URDF 或 Python 常量复制。
- 位姿：`get_pose("left" | "right")`。
- 整机 IK：`solve_ik(left_target_pose=..., right_target_pose=...)` 同时接收左右
  `[x,y,z,roll,pitch,yaw]`，只求解并返回左右各 7 个逻辑关节角。它使用当前 TCP、
  产品限位和 Runtime 当前规划参考/新鲜反馈选择最近支路，不使能电机、不发送 PV、
  不修改 Motion FIFO。新代码推荐显式执行一次 `solve_ik()`，再将结果交给一次
  `set_joint_pv(..., velocity=50)`。
- 状态：`read_state()` 的关节与夹爪快照包含位置、速度、力矩、使能状态以及
  MOS/转子温度；没有有效温度反馈时对应值为 `None`。
- `set_pose(left_target_pose, right_target_pose)` 是末端位姿便捷命令：底层只对
  两侧目标各求一次终点 IK，再按照 Runtime 当前普通 PV 或 MIT 模式安装一份
  14 关节目标。它返回 `None`，没有 motion ID、状态查询或取消接口，也
  不属于 Linear/Circular 路径规划。关节点到点专指输入关节角度的
  `set_joint_pv()`。该入口保留兼容；新代码推荐显式使用 `solve_ik()` 和
  `set_joint_pv()`，以便直接查看和复用关节解。

推荐的 Pose-to-Pose 写法只进行一次整机 IK 和一次普通 PV 提交：

```python
left_q, right_q = robot.solve_ik(
    left_target_pose=left_pose,
    right_target_pose=right_pose,
)
robot.set_joint_pv(left=left_q, right=right_q, velocity=50)
```
- 关节轨迹、Linear 和 Circular 都由 Runtime 返回统一命名空间中的 motion ID，并进入同一个
  原生 FIFO。`get_motion_status(motion_id)` 查询任一任务，`cancel_motion(motion_id)` 只取消
  指定任务，`cancel_all_motions()` 取消全部任务。Python 不生成 ID、不维护 FIFO，也不推断
  完成状态。
- 双臂关节轨迹使用 `move_joint_trajectory()` 一次提交全部时间戳和 14 关节路点并直接返回
  motion ID。PV 模式由 Runtime 生成内部 100 Hz 规划关键点，并在相邻关键点间连续重采样，
  以 500 Hz 下发 PV 轨迹命令；不会把实时 PV 暴露给 Python。2 ms worker 同时执行安全调度，Python
  不重采样或逐帧发送。用户只提交位置和时间戳，不能填写轨迹速度、加速度、jerk 或 PV
  速度限制；MIT 关节轨迹仍直接按五次曲线执行。
- Python 录制回放工具与上述原生轨迹 API 相互独立：录制频率最大为 500 Hz，回放按
  文件时间戳逐点调用普通 `set_joint_pv()` 或 `set_joint_mit()`，不在 Python
  重采样、不插值，
  也不调用 `move_joint_trajectory()`。
- 维护：`clear_motor_faults()` 只清错、不运动；`set_zero()` 把当前位置标定为零点。
- 急停：调用无参数 `estop()` 后底层立即停止控制并失能整机；固定原因从
  `get_health().fault_reason` 读取，且只能通过 `recover()` 解除锁存。
- 恢复：`recover()` 由 C++ Runtime 完成整机清错、双臂健康验证、低速回到已标定零位，
  最后保持整机失能；任一步骤失败都会再次尝试失能并把具体阶段写入 `get_health()` 返回值。
- 重力补偿：`start_gravity_compensation()`、`stop_gravity_compensation()`。
- 双臂协同：PV 或 MIT 模式下调用 `start_bimanual_follow(leader="left")`，底层记录
  启动瞬间的七关节相对位置；随后继续用普通 `set_joint_pv()` 或 `set_joint_mit()` 控制
  主臂，另一侧由 C++ Runtime 自动跟随。普通接口中的从臂数组不会成为从臂目标；实际
  从臂目标由底层相对关系生成。`stop_bimanual_follow()` 会立即在当前模式保持退出位置。

所有关节数量、有限值、URDF 限位、速度和安全检查由 C++ Runtime 统一验证。Python 不重复维护一份产品配置。

`get_pose()` 和全部笛卡尔运动统一使用产品控制点：`with_grippers=True` 时为夹爪中心
`l-tool0/r-tool0`，`with_grippers=False` 时为法兰 `l-link7/r-link7`。SDK 不提供另一套
`get_flange_pose()`，也不在 Python 中换算工具偏移。URDF 中 `tool0` 相对 `link7` 的固定
变换为 `xyz=[-0.004, 0, -0.178] m`、`rpy=[0, 0, 0]`。

笛卡尔位姿统一为 `[x, y, z, roll, pitch, yaw]`，位置单位为米、姿态单位为弧度。
`solve_ik()` 必须同时提供左右 Pose，固定返回
`(left_q[7], right_q[7])`。它只调用一次整机原生 IK，不在 Python 中逐臂或逐点
求解。没有规划参考时，Runtime 使用连接后的新鲜完整反馈作为种子；没有可用反馈时
调用失败并透传原生错误。
`set_pose()` 的 `speed_percent` 范围为 `[1, 100]`；该参数只影响 PV 模式，MIT
模式会忽略。Linear/Circular 使用正数
`duration_s` 按固定 10 ms 周期控制规划关键点数：`duration_s=3` 通常生成 300 段、
301 个 100 Hz 关键点，Runtime 再以 500 Hz 连续重采样执行；真实到位可能因 PV 限速、加速度限制和反馈稳定
确认而更晚。Linear 和 Circular 都保证 2 mm / 0.1 rad 或更细的笛卡尔几何离散；
Circular 的位置圆弧经过 via 点，姿态以最短路 SLERP 经过 via 姿态。SDK 只提交
完整目标，不在 Python 中求 IK、插值或逐帧发送。
`set_pose()` 由 Runtime 基于当前规划参考求最近种子 IK，再按当前普通 PV 或 MIT 模式执行；
双臂目标使用同一份参考并一次提交完整 14 关节目标，不在 Python 中依次发送两个单臂目标。
默认 `speed_percent=50`。Linear/Circular 在底层用一条全局五次时间律生成连续
100 Hz 规划关键点，中间点不再逐点刹停，只有整段最终点制动。Runtime 在相邻关键点间
连续重采样并以 500 Hz 下发 PV 轨迹命令，同时承担 watchdog、安全和反馈工作。
`move_linear_trajectory()` 仍是唯一的 Linear 方法：传 `start_pose/end_pose` 时执行
一条直线；传 `poses=[...]` 时将 2～64 个 Pose 原子规划为一个 Motion ID，并对内部
角点默认使用 10 mm 笛卡尔圆角。短线段由 Runtime 自动缩小圆角，SDK 不计算圆弧、
IK 或插值。`duration_s` 表示每条原始线段的参考时间，因此四个闭环三角形 Pose、
`duration_s=3` 对应总参考时间约 9 秒。
关节轨迹、Linear 和 Circular 进入同一个原生 FIFO。连续的 Linear/Circular 在公共端点且
跟踪误差不超过 0.04 rad 时按计划时刻直接切换下一条，不再额外等待 200 ms 稳定窗口；误差
超出门槛或最后一条运动仍按真实反馈确认到位。Python 不实现轨迹插值、实时回放或队列调度。
路径运行期间调用 `set_pose()` 会失败，必须先等待路径
完成或取消相关 motion。
产品限位、速度约束和真实反馈到位判断全部由 Runtime 完成。
普通 PV 的速度与加速度包络只由 Runtime 根据本次 `speed_percent` 管理；SDK 不公开
`set_max_acceleration()` / `get_max_acceleration()`。完整轨迹只接受位置/路径和时间参数，
轨迹速度、加速度与 jerk 由 Runtime 内部生成且不作为用户接口暴露。
`set_joint_pv()` 与 PV 模式下 `set_pose()` 的 `speed_percent` 始终保持 `1..100`
百分比语义。
对 Linear/Circular，只有 `status.state == "completed"` 表示真实反馈已经稳定到位；`state == "queued"` 表示
任务已完成底层规划并正在等待前序任务，`state == "running"` 且
`progress == 1.0` 仍表示底层正在等待机械臂稳定。`set_pose()` 必须同时传
`left_target_pose`、`right_target_pose`，再加可选速度百分比，不提供运动状态。Linear 统一传 `start_pose` 与 `end_pose`，Circular 统一传
`start_pose`、`via_pose` 与 `end_pose`。显式起点是完整路径的几何起点；如果当前规划参考不在
该位置，Runtime 会把普通 PV 关节点到点接近、真实反馈稳定确认和后续路径作为同一个 motion ID
与 FIFO 项执行。起点确认阈值为 5 mm / 0.035 rad。Runtime 在运动前校验完整接近与路径；
规划期间队尾发生变化或任一路径校验失败时，新任务不会入队，当前任务和已有队列保持不变。

运动术语必须严格区分：`set_joint_pv()` 是输入关节角度的普通 PV 关节点到点；
`set_pose()` 只对末端终点求一次 IK，得到关节角后交给当前普通 PV 或 MIT 模式；
Linear/Circular 才规划笛卡尔路径。`move_joint_trajectory()` 表示带时间戳的
14 关节路点；PV 轨迹由 Runtime 生成 100 Hz 规划关键点并以 500 Hz 连续重采样执行，MIT 关节轨迹仍按 MIT
五次曲线执行。

```python
motion_id = robot.move_joint_trajectory(
    timestamps=[0.0, 2.0],
    left_positions=[[0, 0, 0, 1.5708, 0, 0, 0]] * 2,
    right_positions=[
        [0, 0, 0, 1.5708, 0, 0, 0],
        [-0.7854, -0.7854, 0, 1.5708, 0, 0, 0],
    ],
)
status = robot.get_motion_status(motion_id)
robot.cancel_motion(motion_id)
```

夹爪默认使用保护模式。只有明确需要持续追踪开合度时才使用直驱：

```python
robot.set_grippers(
    left=0,
    right=0,
    gripper_level=10,
    mode="direct",
)
```

直驱只关闭夹爪自身的接触保持、堵转判断和过载退让；电机硬故障、反馈超时、通信降级、transport 故障、急停和失能仍由 Runtime 处理。

`control_mode` 只决定 14 个机械臂关节使用 PV 还是 MIT。左右夹爪的电机协议始终由 Runtime 固定为 MIT；夹爪的 `protected/direct` 只选择防堵转策略，SDK 不根据机械臂模式推断或配置夹爪协议。

Motor 0.10.30 起，`direct` 模式的 Kp/Kd 十倍增益完全由 C++ Runtime 应用；Python 只原样传递开合度、力度等级和模式，不会再次放大增益。`protected` 参数保持不变，`gripper_level=0` 仍表示 Kp/Kd 均为 0。

## 常用命令

```bash
conda activate at
cd ~/Articore-SDK

# 100 Hz 读取，10 Hz 显示
python -m arx_d_can.examples.diagnostics.example_01_read_state \
  --mode continuous \
  --display-hz 10

# 500 Hz 读取性能测试
python -m arx_d_can.examples.diagnostics.example_02_benchmark_read_rate \
  --seconds 15 \
  --hz 500 \
  --cached

# 读取 Runtime health 和具体错误
python -m arx_d_can.examples.diagnostics.example_03_read_health

# 获取左右臂当前产品控制点位姿（有夹爪为 tool0，无夹爪为 link7）
python -m arx_d_can.examples.diagnostics.example_04_read_pose

# 整机清错、低速回到已标定零点，最后失能
python -m arx_d_can.examples.maintenance.example_02_recover_to_zero

# MIT：双臂第 4 关节到 90°
python -m arx_d_can.examples.control.example_04_send_position_mit \
  --left "0,0,0,90,0,0,0" \
  --right "0,0,0,90,0,0,0"

# 快速跟随 MIT：面向遥操和高频控制
# 安全警告：只使用小角度连续目标；请勿提交与当前姿态差异过大的目标
python -m arx_d_can.examples.control.example_17_send_position_mit_fast_follow \
  --left "0,0,0,90,0,0,0" \
  --right "0,0,0,90,0,0,0"
```

完整示例见 [arx_d_can/examples/README.md](arx_d_can/examples/README.md)。

## 架构边界

```text
业务代码
  └─ ArxDCanDualArm
       └─ ArticoreRuntime.create_yunyi(...)
            └─ libarticore_runtime.so
                 └─ C++ MotorBackend + can-left/can-right + 14 关节 + 可选双夹爪
```

SDK 不再包含 `ArxDCanArm`、单臂示例、YAML 电机配置、旧 actuator/driver 包或 Python 动力学实现。
私有目录 `_motor_abi` 只保留产品 Runtime C ABI 的动态库加载、C 结构映射、Python 数据模型和调用转发；
不再向 SDK 暴露 Motor、Controller、寄存器、总线扫描或原生 CLI。
