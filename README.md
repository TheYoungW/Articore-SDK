# Articore-SDK

Yunyi V1.0 双臂 Python SDK。这个仓库只支持完整双臂产品；单臂支持将放在独立项目中。

电机通信、双通道 Runtime、产品配置、限位、安全状态机、夹爪、运动学、动力学和重力补偿均由 `motor-drive-layer` 的 C++ Runtime 负责。Python 只保留双臂业务接口和 ctypes 绑定。

## 安装

```bash
conda activate at
pip install -e .
```

依赖要求为 `motor-drive-layer>=0.10.27`。SDK 加载时同时检查 Runtime ABI 2.24 和 Yunyi 产品级十级夹爪能力，避免误加载旧动态库。URDF 继续随 SDK 分发，用于展示、仿真和外部工具；控制参数不从 Python YAML 读取。

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
    print(robot.safety_health)
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
- 夹爪：`set_grippers(left=0..1000, right=0..1000, gripper_level=1..10)`。
- 状态：`read_state()`、`read_cached_state()`。
  `state.left.arm.enabled` 和 `state.right.arm.enabled` 返回逐关节
  `tuple[bool | None, ...]`；夹爪的 `enabled` 使用相同语义。数据直接来自底层新鲜反馈缓存，
  `None` 表示缺失、过期或无法确认。
- 健康：`safety_health`、`get_fps()`。
- 位姿：`get_pose("left" | "right")`。
- 维护：`clear_motor_faults()` 只清错、不运动；`set_zero()` 把当前位置标定为零点。
- 急停：调用无参数 `estop()` 后底层立即停止控制并失能整机；固定原因从
  `safety_health.fault_reason` 读取，且只能通过 `recover()` 解除锁存。
- 恢复：`recover()` 由 C++ Runtime 完成整机清错、双臂健康验证、低速回到已标定零位，
  最后保持整机失能；任一步骤失败都会再次尝试失能并把具体阶段写入 `safety_health`。
- 重力补偿：`start_gravity_compensation()`、`stop_gravity_compensation()`。

所有关节数量、有限值、URDF 限位、速度和安全检查由 C++ Runtime 统一验证。Python 不重复维护一份产品配置。

## 常用命令

```bash
conda activate at
cd ~/Articore-SDK

# 100 Hz 读取，10 Hz 显示
python -m arx_d_can.examples.example_04_read_state \
  --mode continuous \
  --display-hz 10

# 500 Hz 读取性能测试
python -m arx_d_can.examples.example_09_benchmark_read_rate \
  --seconds 15 \
  --hz 500 \
  --cached

# 查看 Runtime 健康和具体错误
python -m arx_d_can.examples.example_12_diagnose_status

# MIT：双臂第 4 关节到 90°
python -m arx_d_can.examples.example_07_send_position_mit \
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
