# ARX-D-CAN Python SDK

面向达妙电机机械臂的通用 Python SDK，支持单臂、双臂、可选夹爪，以及
DM Device、dm-serial、SocketCAN 和 SocketCAN-FD 通信后端。

SDK 不把公共接口绑定到具体产品：

- `ArxDCanArm` 控制一条独立 CAN 通道上的单臂；
- `ArxDCanDualArm` 组合左右两条独立 CAN 通道；
- 机型差异全部由 YAML 配置描述。

当前双臂默认配置是 Yunyi V1.0，默认通过原厂 DM Device 的 CH0/CH1 通信。以后增加
其他产品时不需要再创建产品专用 Python 类。

## 安装

```bash
python -m pip install .
```

底层通信使用 `motor-drive-layer==0.5.8`。运动学、动力学和末端控制额外需要：

```bash
python -m pip install ".[dynamics]"
```

## 单臂 API

```python
from arx_d_can import ArxDCanArm

arm = ArxDCanArm(
    model="yunyi_v1_0_right",
    control_mode="pv",
)
arm.connect()

try:
    state = arm.read_state()
    print(state.positions)

    arm.enable()                           # 自动配置构造时选择的模式
    arm.send_joint_positions([0.0] * 7)    # 直接发送，不插值
finally:
    arm.close()
```

发送接口职责明确分开：

```python
arm.send_joint_positions(target)                 # 发送当前一帧
arm.move_joint_positions(target, seconds=3.0)    # SDK 插值运动
arm.hold_joint_positions(target)                 # 持续刷新最后目标
```

关节数量不由代码写死，而是来自所选机型配置。当前公开控制模式只有经过验证的
`pv` 和 `mit`。模式在创建机械臂时确定，`enable()` 自动完成配置；使能期间禁止
切换模式。

## 双臂 API

```python
from arx_d_can import ArxDCanDualArm

robot = ArxDCanDualArm(control_mode="pv")
robot.connect()

try:
    robot.enable()
    robot.send_joint_positions(
        left=[0.0] * 7,
        right=[0.0] * 7,
    )
finally:
    robot.close()
```

默认构造使用 Yunyi V1.0 左右臂配置。双臂始终是两个独立单臂对象，不会被展平成
一条包含 14 个关节的 CAN 总线：

```python
print(robot.left)
print(robot.right)

robot.move_joint_positions(
    left=[0.0] * 7,
    right=[0.0] * 7,
    seconds=3.0,
)
```

其他双臂产品可以分别指定左右机型：

```python
robot = ArxDCanDualArm(
    left_model="my_left_arm",
    right_model="my_right_arm",
)
```

双臂关节命令通过底层常驻发送线程在左右通道并行提交；PV 和 MIT 使用相同的
`send_joint_positions()` 接口。每条通道内部仍按关节顺序发送，并在左右通道都完成后
进入下一个控制周期，用户不需要自行创建线程。

## Examples

示例按使用形态组织，而不是按产品名组织：

```text
arx_d_can/examples/
├── single_arm/             # 通用单臂示例
│   ├── example_01_scan_ids.py
│   ├── example_02_read_state.py
│   ├── example_03_clear_faults.py
│   ├── example_04_send_position.py
│   ├── example_05_gripper_open_close.py
│   ├── example_06_benchmark_read_rate.py
│   ├── example_07_send_joint_trajectory.py
│   ├── example_08_return_zero.py
│   ├── example_09_diagnose_status.py
│   ├── example_10_set_zero_current_position.py
│   ├── example_11_record_and_replay_trajectory.py
│   └── example_12_gravity_compensation.py
└── dual_arm/               # 对应的双臂示例，当前默认 Yunyi
    ├── example_01_scan_ids.py
    ├── example_02_read_state.py
    ├── example_03_clear_faults.py
    ├── example_04_send_position.py
    ├── example_05_gripper_open_close.py
    ├── example_06_benchmark_read_rate.py
    ├── example_07_send_joint_trajectory.py
    ├── example_08_return_zero.py
    ├── example_09_diagnose_status.py
    ├── example_10_set_zero_current_position.py
    ├── example_11_record_and_replay_trajectory.py
    └── example_12_gravity_compensation.py
```

单臂示例继续支持 `--arm-model`，因此同一套示例可以用于其他机械臂：

```bash
python -m arx_d_can.examples.single_arm.example_02_read_state \
  --arm-model yunyi_v1_0_right

python -m arx_d_can.examples.single_arm.example_04_send_position \
  --arm-model yunyi_v1_0_right \
  --positions "0,-20,-20,0,0,0,0" \
  --mode pv
```

双臂示例目前直接使用 Yunyi 默认左右配置：

```bash
python -m arx_d_can.examples.dual_arm.example_02_read_state
```

## 夹爪

夹爪接口统一使用 `0～1000` 开合度：

```python
arm.move_gripper(1000)  # 张开
arm.move_gripper(0)     # 闭合

robot.set_grippers(left=500, right=500)
robot.open_grippers()
robot.close_grippers()
```

Yunyi 夹爪固定使用 MIT 模式，默认 `Kp=4.0`。堵转检测、接触后的低刚度保持和持续
过载回退由双臂 C++ 安全运行时的常驻线程执行，不需要 Python 控制线程。双臂原生运行
时启用后，单侧原始夹爪命令会被拒绝，避免绕过整机安全状态机。更换自定义末端时可用
`ArxDCanDualArm(left_gripper=False, right_gripper=False)` 关闭产品夹爪。

`read_state()` 同时公开结构化夹爪控制状态：

```python
state = robot.read_state()
print(state.left_gripper.opening)
print(state.left_gripper.control_state)
print(state.left_gripper.contact_detected)
print(state.left_gripper.overload)
```

## Yunyi 产品配置

Yunyi 左右臂使用一份产品级配置：

```text
arx_d_can/config/yunyi_v1_0.yaml
├── arms.left
└── arms.right
```

左右臂仍是两条独立 CAN 通道。为兼容已有代码，通用单臂 API 仍可使用
`yunyi_v1_0_left` 和 `yunyi_v1_0_right` 两个机型名，但它们会选择同一份产品配置中
对应的一侧。YAML 只在对象创建时解析，发送热路径不会重复加载配置。

## 通信后端

| `transport` | `channel` 示例 | 用途 |
|---|---|---|
| `dm-serial` | `/dev/ttyACM0` | 达妙串口 USB2CAN |
| `dm-device` | `0`、`1` | 原厂 DM-USB2FDCAN Dual 固件 |
| `socketcan` | `can0` | Linux 经典 CAN |
| `socketcanfd` | `can0` | Linux CAN-FD |

Yunyi 默认使用原厂 DM Device，不需要刷写固件，也不需要额外传入通信参数：

```bash
motor-drive-layer-install-dm-device --download

python -m arx_d_can.examples.single_arm.example_02_read_state \
  --arm-model yunyi_v1_0_right
```

`ArxDCanArm(model="yunyi_v1_0_left")` 默认使用 CH0，
`ArxDCanArm(model="yunyi_v1_0_right")` 默认使用 CH1；`ArxDCanDualArm()` 默认同时
打开 CH0 和 CH1。`dm-serial`、SocketCAN 等其他后端只在用户显式覆盖时使用。

Linux SocketCAN 需要先配置接口，例如经典 CAN 1 Mbps：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0
```

SocketCAN 速率由 `ip link` 设置，Python 的 `baud` 不会修改 Linux CAN 接口。

## 安全与通信健康

`ArxDCanDualArm` 在真实 motor-drive-layer Controller 上启用编译进 wheel 的 C++ 安全
运行时。Python 只校验和提交一整批左右臂命令；常驻原生线程使用 `steady_clock` 执行
看门狗、反馈检查、双臂联动安全保持、故障锁存和失能确认，不依赖 Python GIL。状态为
`DISCONNECTED → READY → ENABLED → RUNNING → SAFE_HOLD / FAULT`，`FAULT` 只能通过
检查双通道、TransportHealth、新鲜反馈、电机故障码和物理失能状态的 `recover()` 回到
`READY`，不会自动重新使能。
若整个 Python 进程退出，进程内原生线程也会随之终止；SDK 因此在配置阶段同时写入
`motor_communication_timeout_ms`（Yunyi 默认 500 ms），由电机固件在主机进程消失后
执行最终通信超时失能。

```python
robot = ArxDCanDualArm(transport="dm-device", control_mode="pv")
robot.connect()
health = robot.safety_health
print(health.state)
print(health.fault_reason)
print(health.left_transport)
print(health.right_transport)
print(health.disable_confirmed)
```

双臂运行时要求左右关节命令整体接受或拒绝。连接到原生运行时后，不允许通过
`robot.left.send_joint_positions()` 或 `robot.right.send_joint_positions()` 绕过批量状态机；
应调用 `robot.send_joint_positions(left=..., right=...)`。单臂 `ArxDCanArm` 以及无法直接
批量发送的耦合 MIT 机型继续使用原有 Python 安全路径。

运行实机运动示例时，应先激活 Python 环境再执行 `python -m ...`。如果必须使用
`conda run`，需要添加 `--no-capture-output`：

```bash
conda run --no-capture-output -n at python -m \
  arx_d_can.examples.dual_arm.example_04_send_position \
  --left "0,0,0,90,0,0,0" \
  --right "0,0,0,90,0,0,0" \
  --mode pv
```

普通 `conda run -n ...` 会缓存子进程输出，并可能在 Ctrl+C 时由 Conda 自己截获
中断，使 Python 的 `finally` 清理逻辑没有机会完成。运动控制不能依赖这种运行方式。

- 单次双通道批量发送失败、命令超时或连续反馈失败会进入 `SAFE_HOLD`；
- 安全保持失败、设备掉线、反馈严重过期、电机故障或意外失能会锁存 `FAULT`；左右臂
  联动失能，夹爪按产品配置的 `gripper_fault_action: hold | disable` 保持物体或失能，
  保持失败时会继续尝试失能；
- `read_state()` 只返回新鲜、完整反馈；
- `read_cached_state()` 明确读取最近一次成功反馈；
- `communication_health` 提供结构化通信状态；
- `close()` 默认停止后台任务、失能电机并关闭总线。

```python
health = arm.communication_health
print(health.healthy)
print(health.consecutive_feedback_failures)
print(health.last_error)
```

C++ 安全运行时不是安全认证功能。产品仍需要物理急停、电机侧通信超时，以及垂直负载
场景所需的机械制动或防坠机构。

## 自定义机型

内置机型注册在 `arx_d_can/config/models.yaml`。增加单臂机型时创建自己的硬件 YAML，
然后注册机型名称即可继续使用全部 `single_arm` 示例：

```python
from arx_d_can import ArxDCanArm, available_models

print(available_models())
arm = ArxDCanArm(model="my_arm")
```

本地调试未注册配置时，也可以使用高级参数：

```python
arm = ArxDCanArm(config_path="/path/to/my_arm.yaml")
```

## 开发验证

```bash
python -m pip install ".[dev]"
python -m pytest --import-mode=importlib --rootdir=tests tests
python -m pip wheel --no-deps . --wheel-dir dist
```
