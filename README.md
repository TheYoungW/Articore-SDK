# Articore-SDK

Yunyi V1.0 双臂 Python SDK。这个仓库只支持完整双臂产品；单臂支持将放在独立项目中。

电机通信、双通道 Runtime、产品配置、限位、安全状态机、夹爪、运动学、动力学和重力补偿均由 `motor-drive-layer` 的 C++ Runtime 负责。Python 只保留双臂业务接口和 ctypes 绑定。

## 安装

```bash
conda activate at
pip install -e .
```

当前版本严格依赖 `motor-drive-layer==0.31.0`，并严格要求 Runtime ABI 16.0
(`0x00100000`)；ABI 不一致时直接拒绝加载，不执行旧 ABI 兼容。x86_64 与
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
  50。一次调用提交完整 14 关节最终目标，新目标原子替换旧目标；Runtime 第一帧
  直接发送最终 P，不生成中间 P 点。100% 速度上限为
  `[180,180,180,225,225,225,225] deg/s`，100% 加速度上限为
  `[450,450,900,900,900,900,900] deg/s²`。也可通过
  `set_max_speed(rad_s)` 和 `set_max_acceleration(rad_s2)` 为左右两臂共 14 关节设置
  100% 时的全局基础上限；设置 0 恢复逐关节默认值，对应 get 返回 0。速度按比例
  `s` 缩放，加速度按 `s²` 缩放。Motor V 是最大速度限制，不是目标速度；Runtime
  根据真实剩余距离和 `sqrt(2*a*distance)` 制动关系动态升降 V，到点稳定后切换
  为 V=0 保持。SDK 不生成 P 步进、速度斜坡或轨迹重采样，也不公开 Raw PV 直发。
  Linear/Circular 的时间参数化、速度、加速度和 jerk 完全由 Runtime 管理，
  不向用户暴露 duration 或内部轨迹限制。`set_speed_percent(1..100)` 设置
  Runtime 共享速度百分比；Linear/Circular 在任务提交时捕获当前值。
  `move_pose()`、Linear 和 Circular 要求 PV 产品模式。

普通 PV 的全局速度/加速度配置保存在当前 Runtime 实例中，运动过程中修改时由 Runtime
平滑应用，不需要在每条目标命令前重复设置。正数控制值的实际下发分辨率为 `0.01`。
这些配置约束发送给电机 POS_VEL 模式的 V/A 参数，不是对实测物理速度的安全级硬钳位；
受电机内部控制和机械惯性影响，瞬时实测速度可能高于配置的 V。
- 标准 MIT 使用 `set_joint_mit(...)`，调用方必须显式提供每侧 7 轴的
  `q/dq/tau_ff` 以及 `kp/kd`。`kp/kd` 可传一个标量广播到 14 轴，或直接传
  14 个值。固定原生顺序为 `q, dq, kp, kd, tau_ff, 14`；SDK 不选择默认增益。
  新帧原子覆盖旧帧，Runtime 不插值、不生成轨迹，并由流式命令看门狗保护。
- 快速 MIT 使用 `set_joint_mit_fast(left=..., right=..., velocity=1..100)`，接收
  完整双臂关节角和参考步进速度百分比。Runtime 内部固定 `dq=0`、`tau_ff=0`
  和快速跟随 Kp/Kd；100 对应 5 rad/s。上层应持续提供小角度连续目标。

旧代码迁移：

```python
# 标准 MIT
robot.set_joint_mit(
    left_positions=left_q,
    right_positions=right_q,
    left_velocities=left_dq,
    right_velocities=right_dq,
    kp=kp,
    kd=kd,
    left_feedforward_torques=left_tau_ff,
    right_feedforward_torques=right_tau_ff,
)

# 快速 MIT
robot.set_joint_mit_fast(left=left_q, right=right_q, velocity=50)
```
- 电机电源：`enable()` / `disable()` 操作整机；传入
  `motors=["l-joint4", "r-joint4"]` 时由 C++ Runtime 执行一次原子批量事务。部分使能状态下
  仍提交完整 14 轴目标，底层自动跳过主动失能的电机。
- 原始帧：标准 `set_joint_mit()` 已覆盖显式 MIT 帧能力；旧的
  `submit_raw_mit()` 已删除。用户普通 PV 必须走 `set_joint_pv()` 最终目标路径，
  不公开 Raw/流式 PV 接口。
- 夹爪：`set_grippers(left=0..1000, right=0..1000, gripper_level=0..10, mode="protected" | "direct")`。默认 `protected`；`direct` 会持续追踪目标且不执行夹爪接触保持、堵转判断和过载退让。直驱时应关注夹爪温升、机械过载和被夹物体安全。
- 状态：`read_state()`。
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
  不启动运动。需要显式查看关节解时，可执行一次 `solve_ik()`，再将结果交给一次
  `set_joint_pv(..., velocity=50)`。
- 状态：`read_state()` 的关节与夹爪快照包含位置、速度、力矩、使能状态以及
  MOS/转子温度；没有有效温度反馈时对应值为 `None`。同一状态中的
  `motion_arrived` 表示当前有限笛卡尔运动已经按真实反馈稳定到位。
- 有限笛卡尔运动：`move_pose(side=..., target_pose=...)`、
  `move_linear(...)`、`move_circular(...)`。三者都是非阻塞发送接口，成功时返回
  `None`；Runtime 同一时间只接受一条有限笛卡尔运动，不公开 motion ID、FIFO 或
  单任务查询/取消。用户程序自行轮询 `read_state().motion_arrived`，并结合
  `get_health()` 处理故障和自己的超时策略。`stop_motion()` 停止当前有限运动。
- 关节点到点轨迹已从 Runtime ABI 和 SDK 删除；关节目标使用普通
  `set_joint_pv()`、`set_joint_mit()` 或 `set_joint_mit_fast()`。
- Python 录制回放工具与笛卡尔轨迹 API 相互独立：录制频率最大为 500 Hz，回放按
  文件时间戳逐点调用普通 `set_joint_pv()` 或快速 `set_joint_mit_fast()`，不在 Python
  重采样或插值。
- 维护：`clear_motor_faults()` 只清错、不运动；`set_zero()` 把当前位置标定为零点。
- 急停：调用无参数 `estop()` 后底层立即停止控制并失能整机；固定原因从
  `get_health().fault_reason` 读取，且只能通过 `recover()` 解除锁存。
- 恢复：`recover()` 由 C++ Runtime 完成整机清错、双臂健康验证、低速回到已标定零位，
  最后保持整机失能；任一步骤失败都会再次尝试失能并把具体阶段写入 `get_health()` 返回值。
- 重力补偿：`start_gravity_compensation()`、`stop_gravity_compensation()`。
- 双臂协同：PV 或 MIT 模式下调用 `start_bimanual_follow(leader="left")`，底层记录
  启动瞬间的七关节相对位置；随后继续用普通 `set_joint_pv()` 或 MIT 接口控制
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
使用 `set_speed_percent(1..100)` / `get_speed_percent()` 设置或读取有限笛卡尔
运动使用的共享速度百分比，默认值为 100。每条 `move_*` 在提交时捕获当前值；
运动开始后再修改不会重新定时当前轨迹。

Linear/Circular 不接收 `duration_s`。Runtime 根据路径距离、共享速度百分比、
速度/加速度限制、IK 关节变化和线性化误差自动选择安全时间。轨迹使用
`s(u)=10u³-15u⁴+6u⁵` 五次时间律，起点和终点的速度、加速度均为零。
内部 trajectory-PV knot 间隔自适应为 4～50 ms，最大相邻关节步长为 0.020 rad，
关节线性化容差为 0.001 rad；Runtime 在 knot 间线性重采样并以 500 Hz 下发。
这些参数都不是 SDK 公共配置。

Linear 的 XYZ 严格沿直线；Circular 生成经过 start/via/end 的定向圆弧；姿态使用
最短路 quaternion SLERP，Circular 的姿态经过 via pose。路径首点只使用当前
规划关节作为 IK seed，后续点只使用上一个 IK 解，不尝试随机、回退或外推 seed。
IK 优先 XYZ，位置容差为 0.0005 m，姿态允许最大约 0.035 rad 残差；Runtime 会在
执行前拒绝超过 0.35 rad 的分支跳变和异常的短距离关节方向抖动。SDK 只提交完整路径并
原样传递 Runtime 错误，不在 Python 中采样、求 IK、插值或重采样。
`move_pose()` 只要求目标位姿，起点由 Runtime 在提交时原子地取当前规划位姿；
`move_linear()` 传 `start_pose/end_pose` 时执行一条直线，省略 `start_pose` 或显式
传 `None` 时也从当前规划位姿开始。SDK 不会先调用 `get_pose()` 再回传起点，避免
读取与提交之间产生竞态。传 `poses=[...]` 时执行 2～64 个 Pose 组成的多段直线。
尖锐角点按独立的 rest-to-rest 五次轨迹段处理，机械臂会在角点减速至停止，
不自动切角、生成圆弧过渡或 blending。`move_circular()` 执行经过
start/via/end 的定向圆弧。当前运动尚未到达时提交下一条 `move_*` 会返回 busy；
应用应等待 `motion_arrived` 后再发送。
产品限位、速度约束和真实反馈到位判断全部由 Runtime 完成。
普通 PV 的速度与加速度包络由 Runtime 根据全局基础上限和本次 `speed_percent` 管理；SDK
只原样传递参数，不重复计算百分比。全局速度最大约 `3.14159 rad/s`，全局加速度最大约
`7.85398 rad/s²`；负数、NaN、无穷大或超限值由 Runtime 拒绝，SDK 不静默裁剪。笛卡尔轨迹只接受路径，
轨迹速度、加速度与 jerk 由 Runtime 内部生成且不作为用户接口暴露。
显式起点是完整路径的几何起点；如果当前规划参考不在该位置，Runtime 会先平滑
接近并确认起点，再执行声明路径。起点确认阈值为 5 mm / 0.035 rad。Runtime 在
运动前校验完整接近与路径；规划或校验失败时不会安装新运动。

运动术语必须严格区分：`set_joint_pv()` 是输入关节角度的普通 PV 关节点到点；
`move_pose()`、`move_linear()`、`move_circular()` 都是 Runtime 规划的有限笛卡尔
轨迹。SDK 不再提供 `set_pose()` 或带时间戳的关节点到点轨迹接口。

```python
robot.set_max_speed(1.5)          # rad/s，100% 时的全局基础上限
robot.set_max_acceleration(3.0)   # rad/s²，100% 时的全局基础上限
robot.set_joint_pv(left=left_q, right=right_q, velocity=50)

assert robot.get_max_speed() == 1.5
assert robot.get_max_acceleration() == 3.0

robot.set_max_speed(0)            # 清除配置，恢复逐关节默认值
robot.set_max_acceleration(0)
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
  --hz 500

# 读取 Runtime health 和具体错误
python -m arx_d_can.examples.diagnostics.example_03_read_health

# 获取左右臂当前产品控制点位姿（有夹爪为 tool0，无夹爪为 link7）
python -m arx_d_can.examples.diagnostics.example_04_read_pose

# 整机清错、低速回到已标定零点，最后失能
python -m arx_d_can.examples.maintenance.example_02_recover_to_zero

# 标准 MIT：显式提供 Kp/Kd，从当前反馈分段移动到较小的演示目标
python -m arx_d_can.examples.control.example_04_send_position_mit \
  --left "0,0,0,20,0,-20,0" \
  --right "0,0,0,20,0,-20,0" \
  --kp 20 \
  --kd 1 \
  --max-step-deg 2 \
  --step-interval 0.5

# 快速 MIT：面向遥操和高频控制
# 安全警告：只使用小角度连续目标；请勿提交与当前姿态差异过大的目标
python -m arx_d_can.examples.control.example_17_send_position_mit_fast \
  --left "0,0,0,90,0,0,0" \
  --right "0,0,0,90,0,0,0" \
  --velocity 50
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
