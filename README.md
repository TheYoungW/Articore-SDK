# Articore-SDK

Yunyi V1.0 双臂 Python SDK。这个仓库只支持完整双臂产品；单臂支持将放在独立项目中。

电机通信、双通道 Runtime、产品配置、限位、安全状态机、夹爪、运动学、动力学和重力补偿均由 `motor-drive-layer` 的 C++ Runtime 负责。Python 只保留双臂业务接口和 ctypes 绑定。

## 安装

```bash
conda activate at
pip install -e .
```

当前版本只依赖 `motor-drive-layer==0.12.8`，x86_64 与 ARM64 由 pip 自动选择对应 wheel，不需要安装其他 Motor、Runtime 或 SocketCAN 包。SDK 严格要求 Runtime ABI 3.6，并检查 C++ 直连 Motor 核心、最大速度唯一 PV 路径、原生笛卡尔运动和产品控制点能力。Runtime 只通过三参数 `articore_runtime_create_yunyi(mode, with_grippers, &runtime)` 创建，不再包含旧工厂兼容分支。PV 参数、500 Hz 控制周期、逐关节到位收敛及底层诊断均由 C++ Runtime 内部管理，SDK 不公开控制频率或 Python 调参接口。URDF 继续随 SDK 分发，用于展示、仿真和外部工具；控制参数不从 Python YAML 读取。

## 最小用法

```python
from arx_d_can import ArxDCanDualArm

robot = ArxDCanDualArm(control_mode="mit", with_grippers=True)
try:
    robot.connect()
    robot.enable()

    robot.set_max_speed(70)
    robot.set_joint_mit(
        left=[0.0] * 7,
        right=[0.0] * 7,
        velocity=30,
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

- 普通 PV 位置：先按需调用 `set_max_speed(0..100)`，再调用不带速度参数的
  `set_joint_pv(left=..., right=...)`。0～100 线性对应 0～2 rad/s reference
  slew；产品默认值为 50，对应 1 rad/s 和 500 Hz 下每周期 0.002 rad；
  100 对应 2 rad/s 和每周期 0.004 rad。达妙 POS_VEL 的 `V` 始终固定为
  3 rad/s，不随百分比缩放，为电机追赶 reference 保留余量。
  SDK 不再提供每条 PV 位置命令的速度参数，也不公开 Raw PV 直发。
- 普通 MIT 位置继续使用 `set_joint_mit()` 的显式 `velocity`；Raw MIT 保留给明确需要
  Kp、Kd、目标速度和前馈力矩的高级控制。
- 电机电源：`enable()` / `disable()` 操作整机；传入
  `motors=["l-joint4", "r-joint4"]` 时由 C++ Runtime 执行一次原子批量事务。部分使能状态下
  仍提交完整 14 轴目标，底层自动跳过主动失能的电机。
- 原始帧：产品 SDK 只保留 `submit_raw_mit()`；PV 必须走 Runtime 步进路径。
- 夹爪：`set_grippers(left=0..1000, right=0..1000, gripper_level=0..10, mode="protected" | "direct")`。默认 `protected`；`direct` 会持续追踪目标且不执行夹爪接触保持、堵转判断和过载退让。直驱时应关注夹爪温升、机械过载和被夹物体安全。
- 状态：`read_state()`、`read_cached_state()`。
  `state.left.arm.enabled` 和 `state.right.arm.enabled` 返回逐关节
  `tuple[bool | None, ...]`；夹爪的 `enabled` 使用相同语义。数据直接来自底层新鲜反馈缓存，
  `None` 表示缺失、过期或无法确认。
- 健康：`get_health()`、`get_fps()`。
- 位姿：`get_pose("left" | "right")`。
- PV 笛卡尔 PTP 可用 `move_pose()` 控制单侧，也可用 `move_poses()` 一次原子提交
  左右两侧；两侧 IK 都成功后，Runtime 才安装一份普通 PV 的 14 关节目标。PTP 返回
  `None`，没有 motion ID、状态查询或取消接口。只有 `move_linear()`、`move_circular()` 返回
  motion ID 并进入原生 FIFO；`cartesian_motion_status` 返回最近提交的路径任务，
  `get_cartesian_motion_status(motion_id)` 查询指定路径，`cancel_cartesian_motion()`
  取消当前 Linear/Circular 并清空路径队列。
- 双臂关节轨迹使用 `start_trajectory()` 一次提交全部时间戳和 14 关节路点；
  `trajectory_status` 查询原生执行状态，`cancel_trajectory()` 取消。五次多项式生成、
  校验和 500 Hz 执行全部位于 C++ Runtime，Python 不做 100 Hz 重采样或逐帧发送。
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

笛卡尔位姿统一为 `[x, y, z, roll, pitch, yaw]`，位置单位为米、姿态单位为弧度，
`speed_percent` 范围为 `(0, 100]`。SDK 只提交完整目标，不在 Python 中求 IK、插值或逐帧
发送。PTP 由 Runtime 基于当前规划参考求最近种子 IK，再按普通 PV reference 步进；
双臂 PTP 使用同一份参考并一次提交完整 14 关节目标，不在 Python 中依次发送两个单臂目标。
默认 `speed_percent=50`。Linear 和 Circular 在底层生成直线/圆弧与 SLERP 样本并连续
求 IK，再由 500 Hz worker 每 2 ms 采样发送 PV 帧，路径速度上限保持 3 rad/s。
只有 Linear/Circular 进入原生 FIFO，前一条经反馈确认到位后才开始下一条；Python
不实现轨迹插值、实时回放或队列调度。路径运行期间调用 PTP 会失败，必须先等待路径
完成或调用 `cancel_cartesian_motion()`。
产品限位、速度约束和真实反馈到位判断全部由 Runtime 完成。
对 Linear/Circular，只有 `status.state == "completed"` 表示真实反馈已经稳定到位；`state == "queued"` 表示
任务已完成底层规划并正在等待前序任务，`state == "running"` 且
`progress == 1.0` 仍表示底层正在等待机械臂稳定。PTP 只传单侧 `target_pose`，或同时传
`left_target_pose`、`right_target_pose`，再加可选速度百分比，不提供运动状态。Linear 统一传 `start_pose` 与 `end_pose`，Circular 统一传
`start_pose`、`via_pose` 与 `end_pose`。显式起点是完整路径的几何起点；如果当前规划参考不在
该位置，Runtime 会把普通 PV PTP 接近、真实反馈稳定确认和后续路径作为同一个 motion ID
与 FIFO 项执行。起点确认阈值为 5 mm / 0.035 rad。Runtime 在运动前校验完整接近与路径；
规划期间队尾发生变化或任一路径校验失败时，新任务不会入队，当前任务和已有队列保持不变。

运动术语必须严格区分：笛卡尔 PTP 只对终点求一次 IK，得到关节角后执行普通 PV
reference 步进；它不是关节轨迹规划。只有 `start_trajectory()` 才表示带时间戳的 14 关节
五次轨迹。关节轨迹的速度由时间戳决定，`pv_velocity_limits` 只是每轴安全上限，不能用
普通 PV 的 `speed_percent` 替代。

```python
trajectory_id = robot.start_trajectory(
    timestamps=[0.0, 2.0],
    left_positions=[[0, 0, 0, 1.5708, 0, 0, 0]] * 2,
    right_positions=[
        [0, 0, 0, 1.5708, 0, 0, 0],
        [-0.7854, -0.7854, 0, 1.5708, 0, 0, 0],
    ],
    pv_velocity_limits=2.5,
)
status = robot.trajectory_status
robot.cancel_trajectory()
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
  --right "0,0,0,90,0,0,0" \
  --velocity 10
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
