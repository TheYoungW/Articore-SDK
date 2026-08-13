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
python -m pip install -e .
```

底层通信与原生安全运行时使用 `motor-drive-layer>=0.8.3,<0.9`。平台 wheel 已包含对应的
DM_Device 厂商运行库；普通用户不需要另行下载 DM_SDK、执行 DM Device 安装命令或
配置厂商动态库路径。
Linux x86_64 wheel 同时包含 v1.0 和 v1.1，默认使用已完成扫描与重连真机验证的
v1.0；开发诊断时可通过 `MOTOR_DM_DEVICE_ABI=v1.1` 显式选择 v1.1。
运动学、动力学和末端控制额外需要：

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

    arm.enable()                          # 自动配置构造时选择的模式
    arm.move_joint_positions([0.0] * 7)   # 底层平滑移动到目标
finally:
    arm.close()
```

普通点到点运动统一使用底层轨迹接口：

```python
state = arm.move_joint_positions(
    target,
    velocity=1.0,
    profile="min_jerk",
)                                # 阻塞；底层完成整条轨迹后返回
```

`move_joint_positions()` 不在 Python 中插值，轨迹完全由 C++ runtime 执行。
`velocity` 表示实际轨迹速度，单位为 rad/s；省略时使用 SDK 默认轨迹速度。
插值可选 `min_jerk` 或 `linear`。
它是阻塞接口，因此同一线程连续调用 A、B 时一定先完成 A 再开始 B，不存在用户可见
的轨迹队列或异步句柄。Runtime 同一时间只允许一条活动轨迹；轨迹运行期间提交新轨迹
或普通实时命令会直接返回 busy/rejected，不会抢占，也不会进入等待队列。轨迹完成后
底层持续发送终点保持，不会误触发用户命令看门狗。

已经自行生成连续目标的 ROS、遥操作和视觉跟随程序，可以在 PV 模式下使用
`stream_joint_positions()`。MIT 的原始位置、速度、Kp/Kd 和前馈力矩提交不属于
普通用户接口；MIT 点到点运动同样使用 `move_joint_positions()`。流式目标使用
`STREAMING` 生命周期，停止更新后仍受命令看门狗保护。

关节数量不由代码写死，而是来自所选机型配置。当前公开控制模式只有经过验证的
`pv` 和 `mit`。模式在创建机械臂时确定，`enable()` 自动完成配置；使能期间禁止
切换模式。

## 双臂 API

```python
from arx_d_can import ArxDCanDualArm

robot = ArxDCanDualArm()  # 默认 MIT；PV 使用 control_mode="pv"
robot.connect()

try:
    robot.enable()
    robot.move_joint_positions(
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
    velocity=1.0,
    profile="min_jerk",
)
```

其他双臂产品可以分别指定左右机型：

```python
robot = ArxDCanDualArm(
    left_model="my_left_arm",
    right_model="my_right_arm",
)
```

双臂点到点命令由底层在同一时间轴插值，并通过常驻发送线程向左右通道并行提交。
左右 14 个关节作为一个完整批次接受或拒绝，用户不需要自行创建发送线程。
PV 高级流式控制使用 `stream_joint_positions()`；MIT 原始提交仅供 SDK 内部控制器使用。

## 重力补偿

重力补偿只要求用户创建 MIT 模式机械臂，增益过渡、反馈缓存和安全退出由控制器处理：

```python
from arx_d_can import ArxDCanArm, GravityCompensationMode

arm = ArxDCanArm(model="yunyi_v1_0_right", control_mode="mit")

with GravityCompensationMode(arm) as gravity:
    gravity.run()  # 按 Ctrl+C 后恢复当前位置保持并失能
```

Python 默认以 100 Hz 根据原生反馈缓存计算 URDF 重力矩，motor Runtime 仍以机型配置的
500 Hz 发送最新完整 MIT 目标，并负责看门狗、通信故障、安全保持和夹爪防堵转。双臂
控制器会把左右力矩作为同一个 14 轴批次原子提交，不会按左右顺序发送。模型输出超过
URDF `effort` 时会先限幅，并通过 `sample.limited_joints` 暴露发生限幅的关节。

重力补偿依赖 Pinocchio，请先安装 `.[dynamics]`。公开示例不要求用户填写 Kp/Kd、
补偿比例或发送频率：

```bash
python -m arx_d_can.examples.single_arm.example_12_gravity_compensation
python -m arx_d_can.examples.dual_arm.example_15_gravity_compensation
```

## Examples

示例按使用形态组织，而不是按产品名组织：

```text
arx_d_can/examples/
├── single_arm/             # 通用单臂示例
│   ├── example_01_scan_ids.py
│   ├── example_02_read_state.py
│   ├── example_03_clear_faults.py
│   ├── example_04_send_position.py
│   ├── example_05_set_gripper_opening.py
│   ├── example_06_benchmark_read_rate.py
│   ├── example_07_send_joint_trajectory.py
│   ├── example_08_return_zero.py
│   ├── example_09_diagnose_status.py
│   ├── example_10_set_zero_current_position.py
│   ├── example_11_record_and_replay_trajectory.py
│   └── example_12_gravity_compensation.py
└── dual_arm/               # 对应的双臂示例，当前默认 Yunyi
    ├── example_01_scan_ids.py
    ├── example_02_switch_control_mode.py
    ├── example_03_enable_disable.py
    ├── example_04_read_state.py
    ├── example_05_clear_faults.py
    ├── example_06_send_position_pv.py
    ├── example_07_send_position_mit.py
    ├── example_08_set_gripper_openings.py
    ├── example_09_benchmark_read_rate.py
    ├── example_10_send_joint_trajectory.py
    ├── example_11_return_zero.py
    ├── example_12_diagnose_status.py
    ├── example_13_set_zero_current_position.py
    ├── example_14_record_and_replay_trajectory.py
    └── example_15_gravity_compensation.py
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
python -m arx_d_can.examples.dual_arm.example_04_read_state
```

ID 扫描由 motor-drive-layer 0.8.3 在每条通道的一次连接内批量完成；扫描过程只请求
反馈，结束时不会发送使能、失能或运动控制帧。

默认读取一次；需要以 100 Hz 持续读取时使用：

```bash
python -m arx_d_can.examples.dual_arm.example_04_read_state \
  --mode continuous
```

持续模式固定以 100 Hz 请求反馈，默认只以 10 Hz 刷新终端；任何一次完整反馈失败
都会立即抛出，不会由示例吞掉或累计重试。可用 `--display-hz` 调整显示频率。

## 夹爪

夹爪接口统一使用 `0～1000` 开合度：

```python
arm.set_gripper_opening(1000)

robot.set_gripper_openings(left=1000, right=500)
```

单臂和双臂夹爪 Demo 都要求显式填写目标，不提供隐含的 open/close 动作：

```bash
python -m arx_d_can.examples.single_arm.example_05_set_gripper_opening \
  --opening 1000

python -m arx_d_can.examples.dual_arm.example_08_set_gripper_openings \
  --left-gripper 1000 \
  --right-gripper 500
```

Yunyi 夹爪固定使用 MIT 模式，默认 `Kp=4.0`。堵转检测、接触后的低刚度保持和持续
过载回退由单臂或双臂 C++ 安全运行时的常驻线程执行，不需要 Python 控制线程。双臂原生
运行时启用后，单侧原始夹爪命令会被拒绝，避免绕过整机安全状态机。更换自定义末端时
可用 `ArxDCanArm(enable_gripper=False)`，或
`ArxDCanDualArm(left_gripper=False, right_gripper=False)` 关闭产品夹爪。

`read_state()` 同时公开结构化夹爪控制状态：

```python
state = robot.read_state()
print(state.left_gripper.opening)
print(state.left_gripper.control_state)
print(state.left_gripper.contact_detected)
print(state.left_gripper.overload)
```

单臂使用同一份结构化状态：

```python
gripper = arm.gripper_safety_health
print(gripper.control_state)
print(gripper.contact_detected)
print(gripper.overload)
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

Yunyi 默认使用原厂 DM Device，不需要刷写固件，也不需要额外传入通信参数或安装
厂商动态库。`motor-drive-layer>=0.8.3,<0.9` 的平台 wheel 会自动加载随包提供的运行库；
`MOTOR_DM_DEVICE_LIB` 和下载器只作为 motor-drive-layer 开发、诊断时的回退机制。

达妙官方 macOS v1.1 dylib 声明的最低系统版本为 macOS 26，因此包含该运行库的
wheel 使用 `macosx_26_0` 标签，不声明对更早 macOS 版本兼容。

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

`ArxDCanArm` 和 `ArxDCanDualArm` 在真实 motor-drive-layer Controller 上启用由
motor-drive-layer 0.8.3 提供的 ABI 1.6 `libarticore_runtime`。SDK 初始化时同时要求
`deterministic_disable` 能力。本仓库不再编译 C++；Python 只校验
和提交完整的单臂或双臂命令。常驻原生线程使用
`steady_clock` 执行看门狗、反馈检查、安全保持、故障锁存和失能确认，不依赖 Python GIL。状态为
`DISCONNECTED → READY → ENABLED → RUNNING → SAFE_HOLD / FAULT`，`FAULT` 只能通过
检查所有活动通道、TransportHealth、新鲜反馈、电机故障码和物理失能状态的 `recover()` 回到
`READY`，不会自动重新使能。
若整个 Python 进程退出，进程内原生线程也会随之终止；SDK 因此在配置阶段同时写入
`motor_communication_timeout_ms`（Yunyi 默认 500 ms），由电机固件在主机进程消失后
执行最终通信超时失能。

```python
robot = ArxDCanDualArm(control_mode="pv")
robot.connect()
health = robot.safety_health
print(health.state)
print(health.fault_reason)
print(health.left_transport)
print(health.right_transport)
print(health.disable_confirmed)
```

双臂运行时要求左右关节命令整体接受或拒绝。连接到原生运行时后，不允许通过
`robot.left.stream_joint_positions()` 或 `robot.right.stream_joint_positions()` 绕过批量
状态机；PV 实时跟随应调用 `robot.stream_joint_positions(left=..., right=...)`。
单臂 `ArxDCanArm` 使用只包含一个 Controller 的同一原生运行时。SDK 不再包含
Python 看门狗、安全保持或夹爪防堵转执行循环；机械臂和夹爪正常运行时统一由原生
线程以 500 Hz 发送，`SAFE_HOLD/FAULT` 下统一使用 100 Hz。相关职责由
motor-drive-layer 0.8.3 承担。

使能时，SDK 先完成控制模式和电机参数配置，然后只调用一次原生 `runtime.enable()`。
ABI 1.6 Runtime 会并行刷新 CH0/CH1 的失能反馈，读取全部电机当前位置，生成安全保持
目标，并行使能左右通道并确认所有电机均返回新鲜 `ENABLED` 反馈。失败时 Runtime 会
回滚失能全部电机。SDK 不再提前调用左右臂或夹爪的物理使能接口。

普通运行故障会锁存 `FAULT` 并停止活动轨迹，但不会自动让其他健康关节掉电：仍可控制
的机械臂和夹爪继续发送保护保持，夹爪反馈丢失时保留最后安全夹持目标。只有用户明确
调用 `disable()`（或执行明确的急停策略）才请求全部电机物理失能。

使能失败会抛出 `NativeEnableError`。诊断程序应读取结构化报告，不要解析错误字符串：

```python
from arx_d_can import NativeEnableError

try:
    robot.enable()
except NativeEnableError as exc:
    report = exc.report
    print(report.enabled_count, report.expected_count)
    print(report.missing_motors)
    print(report.motors)
    print(report.disable_confirmed)
    raise
```

也可以通过 `robot.last_enable_report` 读取最近一次使能报告。`disable()` 在 `FAULT`
状态下仍会尝试物理失能全部电机，但不会清除故障锁存；`recover()` 只有在物理失能、
反馈新鲜且通信健康均得到确认后才会回到 `READY`。

ABI 1.6 的 `disable()` 和 `close()` 使用同一个确定性失能事务：停止接收新命令，等待
在途批次完成，建立 ControllerGroup 与 USB/CAN 队列屏障，并行失能 CH0/CH1 并确认
所有电机的新鲜失能反馈；第一轮未确认的电机只会被定向重发一次。正常关闭不再依赖
电机通信超时。失能确认失败时会抛出携带 `DisableReport` 的 `NativeDisableError`：

```python
from arx_d_can import NativeDisableError

try:
    robot.close()
except NativeDisableError as exc:
    print(exc.report.missing_motors)
    print(exc.report.motors)
    raise
```

此时 Runtime、ControllerGroup、Controller 和 Transport 均保持有效，不会在 `finally`
中继续释放；排查缺失电机后可以再次调用 `close()`。成功关闭严格按照 Runtime →
ControllerGroup → Controller/Transport 的顺序执行。`disable()` 调用后或 `close()`
失败时，也可以通过 `robot.last_disable_report` 读取最近一次失能报告。

运行实机运动示例时，应先激活 Python 环境再执行 `python -m ...`。如果必须使用
`conda run`，需要添加 `--no-capture-output`：

```bash
conda run --no-capture-output -n at python -m \
  arx_d_can.examples.dual_arm.example_06_send_position_pv \
  --left "0,0,0,90,0,0,0" \
  --right "0,0,0,90,0,0,0" \
  --velocity 200
```

PV 的 `--velocity` 是 0～400 的产品速度档位，必须明确填写。它与 URDF 最大速度
相互独立：100、200、400 分别对应
`[0.5, 0.5, 0.825, 0.825, 1.575, 1.575, 1.575]`、
`[1.0, 1.0, 1.65, 1.65, 3.15, 3.15, 3.15]`、
`[2.0, 2.0, 3.3, 3.3, 6.3, 6.3, 6.3] rad/s`。URDF/YAML `vlim`
只作为绝对安全上限，`velocity_range` 只作为协议缩放量程；0 不执行运动。

MIT 示例只接收左右臂目标角度，控制参数由机型配置统一管理：

```bash
conda run --no-capture-output -n at python -m \
  arx_d_can.examples.dual_arm.example_07_send_position_mit \
  --left "0,0,0,90,0,0,0" \
  --right "0,0,0,90,0,0,0"
```

MIT 示例发送标准的直接位置命令，不做轨迹插值，也不暴露速度参数。MIT 目标速度和
前馈力矩均为零，Kp/Kd 读取机型 YAML。需要平滑插值时使用 example 10。

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
