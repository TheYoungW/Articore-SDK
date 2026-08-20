# Articore-SDK

Yunyi V1.0 双臂 Python SDK。这个仓库只支持完整双臂产品；单臂支持将放在独立项目中。

电机通信、双通道 Runtime、产品配置、限位、安全状态机、夹爪、运动学、动力学和重力补偿均由 `motor-drive-layer` 的 C++ Runtime 负责。Python 只保留双臂业务接口和 ctypes 绑定。

## 安装

```bash
conda activate at
pip install -e .
```

依赖要求为 `motor-drive-layer>=0.10.31`。SDK 加载时检查 Runtime ABI 2.30，以及夹爪和三种原生笛卡尔运动能力，避免误加载旧动态库。URDF 继续随 SDK 分发，用于展示、仿真和外部工具；控制参数不从 Python YAML 读取。

## 最小用法

```python
from arx_d_can import ArxDCanDualArm

robot = ArxDCanDualArm(control_mode="mit", with_grippers=True)
try:
    robot.connect()
    robot.enable()

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

- 普通位置：`set_joint_mit()`、`set_joint_pv()`，`velocity` 为 0～100 档位。
- 电机电源：`enable()` / `disable()` 操作整机；传入
  `motors=["l-joint4", "r-joint4"]` 时由 C++ Runtime 执行一次原子批量事务。部分使能状态下
  仍提交完整 14 轴目标，底层自动跳过主动失能的电机。
- 原始帧：`submit_raw_mit()`、`submit_raw_pv()`。
- 夹爪：`set_grippers(left=0..1000, right=0..1000, gripper_level=0..10, mode="protected" | "direct")`。默认 `protected`；`direct` 会持续追踪目标且不执行夹爪接触保持、堵转判断和过载退让。直驱时应关注夹爪温升、机械过载和被夹物体安全。
- 状态：`read_state()`、`read_cached_state()`。
  `state.left.arm.enabled` 和 `state.right.arm.enabled` 返回逐关节
  `tuple[bool | None, ...]`；夹爪的 `enabled` 使用相同语义。数据直接来自底层新鲜反馈缓存，
  `None` 表示缺失、过期或无法确认。
- 健康：`get_health()`、`get_fps()`。
- 位姿：`get_pose("left" | "right")`。
- PV 笛卡尔运动：`move_pose()`、`move_linear()`、`move_circular()` 一次控制一侧；
  `cartesian_motion_status` 返回 Runtime 的统一异步状态，`cancel_cartesian_motion()` 取消当前轨迹并保持最后参考位置。
- 维护：`clear_motor_faults()` 只清错、不运动；`set_zero()` 把当前位置标定为零点。
- 急停：调用无参数 `estop()` 后底层立即停止控制并失能整机；固定原因从
  `get_health().fault_reason` 读取，且只能通过 `recover()` 解除锁存。
- 恢复：`recover()` 由 C++ Runtime 完成整机清错、双臂健康验证、低速回到已标定零位，
  最后保持整机失能；任一步骤失败都会再次尝试失能并把具体阶段写入 `get_health()` 返回值。
- 重力补偿：`start_gravity_compensation()`、`stop_gravity_compensation()`。

所有关节数量、有限值、URDF 限位、速度和安全检查由 C++ Runtime 统一验证。Python 不重复维护一份产品配置。

笛卡尔位姿统一为 `[x, y, z, roll, pitch, yaw]`，位置单位为米、姿态单位为弧度，
`speed_percent` 范围为 `(0, 100]`。IK、五次轨迹、直线/圆弧插值、限位和到位判断均在 Runtime 中完成。
只有 `status.state == "completed"` 表示真实反馈已经稳定到位；`state == "running"` 且
`progress == 1.0` 仍表示底层正在等待机械臂稳定。当前接口不是左右臂原子运动，SDK 不会用两次单侧调用伪装同步双臂规划。

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

# 获取左右臂当前法兰位姿
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
            └─ libarticore_runtime.so / libmotor_abi.so
                 └─ can-left + can-right + 14 关节 + 可选双夹爪
```

SDK 不再包含 `ArxDCanArm`、单臂示例、YAML 电机配置、旧 actuator/driver 包或 Python 动力学实现。
私有目录 `_motor_abi` 也只保留产品 Runtime 的动态库加载、C 结构映射、Python 数据模型和调用转发；
不再向 SDK 暴露 Motor、Controller、寄存器、总线扫描或原生 CLI。
