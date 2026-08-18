# ARX-D-CAN Python SDK

面向达妙电机机械臂的通用 Python SDK，支持单臂、双臂、可选夹爪，以及
DM Device、dm-serial、SocketCAN 和 SocketCAN-FD 通信后端。

SDK 不把公共接口绑定到具体产品：

- `ArxDCanArm` 控制一条独立 CAN 通道上的单臂；
- `ArxDCanDualArm` 组合左右两条独立 CAN 通道；
- 机型差异全部由 YAML 配置描述。

当前双臂默认配置是 Yunyi V1.0，默认通过两块刷入 `gs_usb` 固件的
DM-USB2FDCAN Dual，以 SocketCAN-FD 的 `can-left`/`can-right` 分别连接左右臂。以后增加
其他产品时不需要再创建产品专用 Python 类。

## 安装

```bash
python -m pip install -e .
```

底层通信与原生安全运行时使用 `motor-drive-layer==0.10.13`（Motor ABI
0.5.0-cpp，Runtime ABI 2.8）。0.10.13 官方预编译包面向 Linux x86_64 和 ARM64；
Wheel 已包含 DM Device 运行库，
并在私有 `lib/robotics/` 中携带 Pinocchio、Boost.Serialization 及所需 C++ 运行库。
普通用户不需要另行下载 DM_SDK，也不需要配置系统 Pinocchio 或 ROS 动态库路径。
内置 Yunyi 模型的 FK、IK、Jacobian 和刚体动力学不需要安装 Pinocchio；用于控制计算的
产品模型和 Pinocchio C++ 实现都在私有 Runtime 中。SDK 仍保留 Yunyi URDF，供可视化、
机器人描述和外部工具使用，但不把它作为内置动力学或控制限位的数据源。只有加载自定义
URDF 或使用迁移期旧运动学/轨迹接口时才需要：

```bash
python -m pip install ".[legacy-models]"
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
    arm.set_joint_pv([0.0] * 7)           # Runtime 在底层平滑推进目标
finally:
    arm.close()
```

普通位置设置按构造模式使用 `set_joint_mit()` 或 `set_joint_pv()`：

```python
arm.set_joint_mit(
    target,
    # velocity 可选；省略时使用当前机型允许的最大统一速度
)
```

用户只提交最终位置和一个统一速度。Runtime 在内部控制周期中限制位置
reference；最新完整目标原子覆盖旧目标，不排队，也不要求 Python 持续刷新。MIT 实际
发送 `dq=0`、`tau=0` 和产品 Kp/Kd；PV 使用相同的 reference 和统一协议速度限制。
SDK 不限制用户重复调用 `set_joint_mit()` / `set_joint_pv()` 的频率：调用快于 Runtime
内部周期时只消费最新目标，调用慢于内部周期时持续保持最近目标，两种情况都不会因
“调用 Hz 超限”而报错。底层调度周期不作为用户侧最大调用频率。
普通 MIT 的统一速度范围为 `(0, 3.49066] rad/s`，即最高 `200°/s`；PV 仍使用机型
URDF/YAML 中的速度上限。省略 `velocity` 时默认采用对应模式与当前机型允许的最大值。
原始 MIT/PV 数据包、逐关节速度、Kp/Kd、前馈力矩和 streaming 生命周期不作为普通
用户 API 公开，只供 SDK 内部重力补偿等高级控制器使用。

关节数量不由代码写死，而是来自所选机型配置。当前公开控制模式只有经过验证的
`pv` 和 `mit`。模式在创建机械臂时确定，`enable()` 自动完成配置；使能期间禁止
切换模式。

## 原生运动学与动力学

内置产品通过薄 NumPy API 调用 Runtime 私有模型，不向 SDK 暴露精确 URDF、质量、
惯量、质心或 Pinocchio 类型：

```python
import numpy as np
from arx_d_can.kinematics import load_native_robot_model

with load_native_robot_model("yunyi_v1_0_right") as model:
    q = np.zeros(model.dof)
    position, rotation, transform = model.fk(q)
    jacobian = model.jacobian(q)
    gravity = model.gravity(q)
    mass = model.mass_matrix(q)
    torque = model.rnea(q, np.zeros(7), np.zeros(7))
```

同一对象还提供 `ik()`、`coriolis_matrix()`、`nonlinear_effects()` 和 `aba()`。
SDK 的 `dynamics` 命名空间只导出原生模型薄封装，不再包含 Python 动力学算法，也不读取
Yunyi URDF 执行计算。旧 Pinocchio 运动学与笛卡尔轨迹函数只作为自定义模型迁移路径保留，并
通过 `.[legacy-models]` 作为可选依赖。

## 双臂 API

```python
from arx_d_can import ArxDCanDualArm

robot = ArxDCanDualArm()  # 默认 MIT；PV 使用 control_mode="pv"
robot.connect()

try:
    robot.enable()
    robot.set_joint_mit(
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
```

其他双臂产品可以分别指定左右机型：

```python
robot = ArxDCanDualArm(
    left_model="my_left_arm",
    right_model="my_right_arm",
)
```

左右 14 个关节作为一个完整批次接受或拒绝，并使用一个统一的 `velocity`；用户不需要
自行创建发送线程。

双臂类同时公开 `submit_raw_mit()`，供遥操作控制器等需要逐帧设置
`q/dq/Kp/Kd/tau_ff` 的高级应用使用，但不在普通用户 example 中调用。该接口保持
左右 14 轴原子更新和 Runtime 流式看门狗语义。SDK 成功热路径只校验/转换数组，并
非阻塞写入容量为一的 latest-target mailbox，不读取 Python 反馈。Runtime ABI 2.6 在每个
真实发送周期使用最新原生 q/dq 重新计算完整 MIT 合成力矩，把它限制在逐关节
配置的完整 `torque_limit` 内；超限时同比缩小该关节的 Kp、Kd 和 `tau_ff`，未超限时不修改命令。

## 重力补偿

Yunyi 重力补偿由 Runtime 原生控制循环管理。单臂可直接使用高层接口：

```python
from arx_d_can import ArxDCanArm

arm = ArxDCanArm(model="yunyi_v1_0_right", control_mode="mit")
arm.connect()
arm.enable()
arm.start_gravity_compensation(transition_ms=1000)
print(arm.gravity_compensation_status)
arm.stop_gravity_compensation()
arm.close()
```

双臂可使用同样的 `start_gravity_compensation()`、`stop_gravity_compensation()` 和
`gravity_compensation_status`，也可使用负责连接、使能和安全退出的兼容上下文：

```python
from arx_d_can import ArxDCanDualArm, DualArmGravityCompensationMode

robot = ArxDCanDualArm(control_mode="mit")

with DualArmGravityCompensationMode(robot) as gravity:
    gravity.run()  # 按 Ctrl+C 后原子停止并失能双臂
```

SDK 在 `runtime.connect()` 前为每个活动侧绑定 `yunyi_v1_0` 原生产品模型。启动时
Runtime 独占机械臂输出：渐入阶段降低 MIT Kp/Kd 并增加重力前馈，进入 `ACTIVE` 后
发送 `Kp=0`、`Kd=0`、`dq=0` 和 `tau_ff=gravity(q)`。最终合力矩仍经过逐周期限制器。
`DualArmGravityCompensationMode.step()` 仅用于读取反馈和最终实际发送的重力前馈，
不会从 Python 提交控制命令。重力补偿期间普通 MIT/PV 指令会被 Runtime 拒绝，夹爪
命令仍可使用。

停止重力补偿会平滑降低重力前馈、恢复产品 MIT Kp/Kd，并回到停止瞬间当前位置保持；
它不会自动失能。`DualArmGravityCompensationMode` 在退出上下文后会继续执行失能和释放
连接。`gravity_compensation_status` 提供阶段、渐变进度、控制周期数、参与关节和经过
力矩限制后的实际前馈。

原生重力补偿仍属于需要真机验收的力矩控制功能。首次运行应使用机械支撑或悬挂、
物理急停和无载荷环境，并在自定义 YAML 的各关节 `effort_limit` 中先填写较低限制；
确认左右侧绑定、`joint1`～`joint7` 顺序和重力力矩方向后，再逐步提高限制。应分别在
零位、中间位和接近限位姿态检查，不能把 Runtime 逐周期力矩限制当作安全认证功能。

内置 Yunyi 重力补偿不依赖 Python Pinocchio。公开示例不要求用户填写 Kp/Kd、补偿比例或发送频率：

```bash
python -m arx_d_can.examples.single_arm.example_11_gravity_compensation \
  --arm-model yunyi_v1_0_right
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
│   ├── example_08_return_zero.py
│   ├── example_09_diagnose_status.py
│   ├── example_10_set_zero_current_position.py
│   └── example_11_gravity_compensation.py
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
    ├── example_11_return_zero.py
    ├── example_12_diagnose_status.py
    ├── example_13_set_zero_current_position.py
    ├── example_15_gravity_compensation.py
    ├── example_16_record_gravity_trajectory.py
    └── example_17_replay_trajectory.py
```

双臂重力示教回放从使能开始始终使用同一种 raw PV 或 raw MIT，由 Runtime 自动调度
完整左右臂目标，不在普通接口与 raw 接口之间切换。回放提供 `none`、`linear`、
`quintic` 三种插值方式；MIT 使用插值位置、`dq=0`、产品 YAML Kp/Kd 和
`tau_ff=0`；
其中 `none` 是零阶保持，`quintic` 使用
`10u³ - 15u⁴ + 6u⁵` 五次 S 曲线。

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

ID 扫描由 motor-drive-layer 0.10.13 在每条通道的一次连接内批量完成；扫描过程只请求
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
from arx_d_can import GripperForceLevel

arm.set_gripper_opening(1000)

robot.set_gripper_openings(left=1000, right=500)
robot.set_gripper_openings(
    left=0,
    right=0,
    speed=500,  # (0, 1000]；1000 对应产品最大标定速度
    force_level=GripperForceLevel.LEVEL_5,
)
```

按次命令只公开开合度、归一化速度和 `LEVEL_1`～`LEVEL_10` 十档产品力等级；
1 最轻、5 默认、10 最强。每档映射完整的接触、过载、
运动增益和保持增益标定；普通用户不能直接修改 MIT Kp/Kd、堵转窗口或回退距离。

单臂和双臂夹爪 Demo 都要求显式填写目标，不提供隐含的 open/close 动作：

```bash
python -m arx_d_can.examples.single_arm.example_05_set_gripper_opening \
  --opening 1000 \
  --speed 1000 \
  --force-level 5

python -m arx_d_can.examples.dual_arm.example_08_set_gripper_openings \
  --left-gripper 1000 \
  --right-gripper 500 \
  --speed 1000 \
  --force-level 5
```

最大速度、10 档力闭合双臂夹爪：

```bash
python -m arx_d_can.examples.dual_arm.example_08_set_gripper_openings \
  --left-gripper 0 \
  --right-gripper 0 \
  --speed 1000 \
  --force-level 10
```

Yunyi 夹爪在连接前绑定 motor 内置的 `yunyi_gripper_v1` 产品 profile。开合映射、
最大速度、十档力控、接触/堵转/过载检测、保持增益和回退策略均由底层统一标定，SDK
不再保存或传递这些参数。双臂原生
运行时启用后，单侧原始夹爪命令会被拒绝，避免绕过整机安全状态机。更换自定义末端时
可用 `ArxDCanArm(enable_gripper=False)`，或
`ArxDCanDualArm(left_gripper=False, right_gripper=False)` 关闭产品夹爪。

公开 `speed=1000` 表示所绑定产品 profile 的最大标定速度。Yunyi 原装夹爪在
`yunyi_gripper_v1` 中对应 10 rad/s；`speed=500` 对应旧版最大速度 5 rad/s。
张开和闭合均由 Runtime 生成受控速度斜坡。正常夹爪控制跟随整机底层调度，不使用
Python 控制线程。

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
├── gripper_profile: yunyi_gripper_v1
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

Yunyi 默认使用两块刷入 `gs_usb` 固件的 DM-USB2FDCAN Dual。Linux 负责配置 CAN-FD
接口，SDK 默认打开 `can-left`/`can-right` 并显式启用 BRS。原厂 `dm-device` 后端继续保留，
需要时可以在构造对象时显式覆盖；`motor-drive-layer==0.10.13` 的 Linux wheel 会自动
加载随包提供的厂商运行库。

0.10.13 不提供官方 macOS 或 Windows 预编译包。

`ArxDCanArm(model="yunyi_v1_0_left")` 默认使用 `can-left`，
`ArxDCanArm(model="yunyi_v1_0_right")` 默认使用 `can-right`；`ArxDCanDualArm()` 默认同时
打开这两个 SocketCAN-FD 接口。原厂 `dm-device` 和 `dm-serial` 后端仍可显式选择。

DM Device 默认正式使用 CAN-FD+BRS，仲裁速率为 1 Mbps、数据速率为 5 Mbps；5 Mbps
数据段的 87.5% 采样点由 motor 底层固定配置，SDK 不提供用户参数。达妙电机必须预先
设置为 `CAN_BR=9`，否则无法与该默认模式通信。SDK 内部保持以下调用：

```python
Controller.from_dm_device(
    device="usb2canfd-dual",
    channel=channel,
    bitrate=1_000_000,
    data_bitrate=5_000_000,
)
```

需要经典 CAN 兼容时，应在 motor 底层显式将 `bitrate` 与 `data_bitrate` 设为相同值；
普通 Yunyi 用户无需接触这些参数。

Linux SocketCAN 需要先配置接口，例如经典 CAN 1 Mbps：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0
```

SocketCAN 速率由 `ip link` 设置，Python 的 `baud` 不会修改 Linux CAN 接口。
Yunyi 电机需要设置为 `CAN_BR=9`。两块适配器首次安装时，根据序列号和实际使用的
通道号安装热插拔规则：

```bash
sudo ./deploy/linux/install-yunyi-canfd.sh \
  LEFT_ADAPTER_SERIAL LEFT_CHANNEL \
  RIGHT_ADAPTER_SERIAL RIGHT_CHANNEL
```

安装后，udev 会把两路稳定命名为 `can-left`、`can-right`；systemd 会在开机和每次
USB 重新插入时自动设置 1 Mbps 仲裁速率、5 Mbps 数据速率、CAN-FD 和
`txqueuelen=1000`，无需再次手工运行 `ip link`。适配器具有两个物理通道，通道号使用
`0` 或 `1`。可以通过 `udevadm info /sys/class/net/canX` 查看序列号，通过
`cat /sys/class/net/canX/dev_port` 查看通道号。

SDK 会显式调用 `Controller.from_socketcanfd(channel, enable_brs=True)`，确保帧包含
`CANFD_BRS`。部分 `gs_usb` 固件不支持 `berr-reporting` 或 `restart-ms`，因此默认
配置命令不要求这两个选项。

## 安全与通信健康

`ArxDCanArm` 和 `ArxDCanDualArm` 在真实 motor-drive-layer Controller 上启用由
motor-drive-layer 0.10.13 提供的正式 Python `ArticoreRuntime`。Runtime ABI、capability、
ctypes 结构、函数签名、native 句柄所有权和报告转换全部由 motor-drive-layer 维护，
Articore-SDK 只提供机器人产品配置并提交完整的单臂或双臂命令。常驻原生线程使用
`steady_clock` 执行看门狗、反馈检查、安全保持、故障锁存和失能确认，不依赖 Python GIL。状态为
`DISCONNECTED → READY → ENABLED → RUNNING → SAFE_HOLD / FAULT`，`FAULT` 只能通过
检查所有活动通道、`RuntimeTransportHealth`、新鲜反馈、电机故障码和物理失能状态的 `recover()` 回到
`READY`，不会自动重新使能。

ABI 2.4 的 `connect()` 会并行获取全部已配置关节和已安装夹爪的新鲜反馈，完整反馈
屏障通过后才进入 `READY`；因此连接成功后可以立即调用 `read_state()` 或
`read_cached_state()`。`READY` 状态的低频反馈刷新同样由 Runtime 负责，SDK 不会绕过
Runtime 主动请求反馈，也不会用零值掩盖缺失电机。
只读取或诊断时使用 `connect(read_only=True)`：该连接仍会创建反馈 Runtime，但不会
写入 MIT/PV 模式、通信看门狗或其他电机寄存器。官方读取、读取频率和诊断示例均使用
该路径。只读 Runtime 持有 Motor lease，因此不能在同一连接上直接调用 `enable()`；
需要控制时必须先 `close()`，再用普通 `connect()` 重新连接后调用 `enable()`。
0.10.12 修复了寄存器写 ACK 在 `send()` 返回前到达时被误判为旧响应的竞态，覆盖
`ensure_mode()` 和 `set_can_timeout_ms()`。SDK 仍保留 fail-closed 寄存器读回保护：
只有模式寄存器 10 或看门狗寄存器 9 与目标值完全一致时才接受写 ACK 异常，读回失败
或数值不一致仍然终止配置。
0.10.13 将 SocketCAN 和 SocketCAN-FD socket 设为非阻塞。发送队列满时默认最多等待
20 ms；`EAGAIN`、`EWOULDBLOCK` 或 `ENOBUFS` 不再让 Controller worker 永久阻塞。
需要调整时可设置 `MOTOR_DRIVE_LAYER_SOCKETCAN_SEND_TIMEOUT_MS`，有效范围为
1～60000 ms。超时会作为明确的 transport fault 返回，并写入
`RuntimeTransportHealth.send_errors` 和 `last_error`；双臂一侧队列堵塞时，批次会在
有限时间内失败并进入 Runtime 安全状态机，另一侧 worker、`close_bus()` 和 Runtime
shutdown 不会被无限拖住。适配器固件、gs_usb、USB 供电或 TX URB 停止仍可能触发
底层队列堵塞，但不会再被软件放大成永久卡死。
SDK 的通用反馈新鲜度窗口默认为 50 ms；Yunyi 双通道产品配置使用 300 ms，
反馈健康检查仍为 100 Hz。缺失或超过窗口的反馈会累计到结构化健康诊断，但不会仅因
反馈超时把运行中的 Runtime 切换到 `SAFE_HOLD` 或 `FAULT`。
连接失败会抛出携带 `ConnectReport` 的 `RuntimeTransactionError`。诊断代码应直接读取
通道、电机和缺失反馈字段，不要解析异常字符串：

```python
from arx_d_can import RuntimeTransactionError

try:
    robot.connect()
except RuntimeTransactionError as exc:
    report = exc.report
    print(report.error_code, report.received_count, report.expected_count)
    print(report.channels)
    print(report.motors)
    raise
```

0.10.10 保留了 DM Device 回调入口复制完整帧并保留物理 channel 的行为，底层同时校验 channel、
仲裁 ID 和 payload CAN ID；与反馈速度明显不相容的单帧位置跳变会在进入缓存前丢弃，
继续保留上一帧，不触发全局 `FAULT` 或失能。SDK 不重复实现这些过滤，
`read_cached_state()` 的调用方式保持不变。底层完整性统计仅供内部硬件诊断使用，不加入
普通机器人接口。
若整个 Python 进程退出，进程内原生线程也会随之终止；SDK 因此在配置阶段同时写入
`motor_communication_timeout_ms`（Yunyi 默认 500 ms），由电机固件在主机进程消失后
执行最终通信超时失能。

SDK 在 `runtime.connect()` 前根据完整的 URDF 标定边界生成命令限位和原生产品绑定，
不再在两端额外
扣除 1°。SDK 不实现接近限位减速；超出标定位置范围的目标会在发送前被拒绝，范围内的运动速度
由调用者指定。Runtime 同时校验命令速度和力矩限位。motor-drive-layer 0.10.13 不再把反馈
位置、速度或力矩与命令限位比较，因此实际反馈轻微越界不会单独触发 `FAULT`、
`SAFE_HOLD` 或失能。原生重力补偿直接使用有限、已使能的实际七轴反馈，不经过 Python
保持目标。首次普通 MIT/PV 命令即使从超出配置硬限位的实际
反馈位置开始，也会由 Runtime 连续向合法目标推进，不会仅因此触发 `FAULT` 或
`SAFE_HOLD`；电机故障码、通信异常、发送失败和无法确认失能仍按原生安全状态机处理。

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

双臂运行时要求左右关节命令整体接受或拒绝。连接到原生运行时后，单侧对象不能绕过
双臂批量状态机；普通位置更新必须调用双臂的 `set_joint_mit()` 或 `set_joint_pv()`。
单臂 `ArxDCanArm` 使用只包含一个 Controller 的同一原生运行时。SDK 不再包含
Python 看门狗、安全保持或夹爪防堵转执行循环；机械臂和夹爪正常运行时统一由原生
线程自动调度，安全保持同样由底层管理。调度周期会根据实际控制器拓扑自动选择，
但不作为用户调用频率的上限。相关职责由 motor-drive-layer 0.10.13 承担。Runtime ABI
2.6 在两侧 Controller 均报告 `socketcanfd + can_fd + can_fd_brs` 时允许原生 Runtime
最高运行到 500 Hz。motor-drive-layer 0.10.8 消除了 raw mailbox 提交与物理发送之间的
锁竞争，并让缓存状态读取只使用 MotorHandle 内部快照锁。标准 SDK 公开
`submit_raw_mit()` 已完成双臂 16 台、500 Hz、30 秒真机验收：提交 500.02 Hz，16 台
反馈 497.36～499.36 Hz，两侧收发错误为零，最终 16/16 失能。因此 Yunyi
SocketCAN-FD+BRS 产品配置默认请求 500 Hz。SDK 始终使用 `runtime.control_hz` 获取真实
调度频率，不根据 transport capability 自行覆盖产品配置；其他 transport 仍由 Runtime
按 capability 限频。
Runtime ABI 2.6 在每个原生 MIT 发送周期（包括重复 mailbox 最新目标）根据最新
缓存反馈执行完整 P+D+前馈力矩保护。限制值直接使用用户配置的 `torque_limit`：未超限时
`scale=1.0`，只在超限时同比缩放 Kp、Kd 和 `tau_ff`。过期但仍有效的缓存样本可继续
用于计算，同时由 health 报告其年龄；缓存不存在、数据非法
或明确报告电机未使能时，整批仍会被拒绝。Yunyi 4310 V1.1 的
产品力矩范围仍为 7 N·m；底层 `TMAX=10` 只用于 MIT 报文编码映射，不作为机械 effort 上限。
双臂 MIT 运行时，14 个关节和两个夹爪由同一个 `ControllerGroup` 批次调度；夹爪反馈
年龄直接读取实时 Motor 缓存，SDK 不再增加单独的夹爪发送或反馈刷新路径。

使能时，SDK 先完成控制模式和电机参数配置，然后只调用一次原生 `runtime.enable()`。
ABI 2.4 Runtime 会并行刷新 CH0/CH1 的失能反馈，读取全部电机当前位置，生成安全保持
目标，并行使能左右通道并确认所有电机均返回新鲜 `ENABLED` 反馈。失败时 Runtime 会
回滚失能全部电机。SDK 不再提前调用左右臂或夹爪的物理使能接口。

普通运行故障会锁存 `FAULT` 并停止活动控制目标，但不会自动让其他健康关节掉电：仍可控制
的机械臂和夹爪继续发送保护保持，夹爪反馈丢失时保留最后安全夹持目标。只有用户明确
调用 `disable()`（或执行明确的急停策略）才请求全部电机物理失能。

使能事务失败会抛出 motor-drive-layer 的 `RuntimeTransactionError`。诊断程序应读取
结构化报告，不要解析错误字符串：

```python
from arx_d_can import RuntimeTransactionError

try:
    robot.enable()
except RuntimeTransactionError as exc:
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

ABI 2.4 的 `disable()` 和 `close()` 使用同一个确定性失能事务：停止接收新命令，等待
在途批次完成，建立 ControllerGroup 与 USB/CAN 队列屏障，并行失能 CH0/CH1 并确认
所有电机的新鲜失能反馈；第一轮未确认的电机只会被定向重发一次。正常关闭不再依赖
电机通信超时。失能或关闭事务失败时同样会抛出携带结构化报告的
`RuntimeTransactionError`：

```python
from arx_d_can import RuntimeTransactionError

try:
    robot.close()
except RuntimeTransactionError as exc:
    print(exc.report.missing_motors)
    print(exc.report.motors)
    raise
```

如果 `close()` 未确认全部电机失能，Runtime、ControllerGroup 和 Transport 均保持原样，
不会释放 native 句柄或资源租用；调用方可继续读取结构化报告并重试 `close()`。只有关闭
事务成功后才会按所有权顺序释放资源。

Python native 句柄和资源租用由 motor-drive-layer 统一处理；SDK 始终按照 Runtime →
ControllerGroup → Controller/Transport 的顺序清理。`disable()` 调用后也可以通过
`robot.last_disable_report` 读取最近一次失能报告。

运行实机运动示例时，应先激活 Python 环境再执行 `python -m ...`。如果必须使用
`conda run`，需要添加 `--no-capture-output`：

```bash
conda run --no-capture-output -n at python -m \
  arx_d_can.examples.dual_arm.example_06_send_position_pv \
  --left "0,0,0,90,0,0,0" \
  --right "0,0,0,90,0,0,0" \
  --velocity 60
```

PV 的 `--velocity` 是双臂所有关节共用的最大参考速度，命令行单位为度/秒，必须
明确填写且大于 0。SDK 转换为 rad/s 后一次提交完整双臂目标；Runtime 在原生
实际控制周期内限步，新的调用直接覆盖最终目标，不排队。关节 YAML `vlim` 仍是
绝对安全上限。

MIT 示例只接收左右臂目标角度，控制参数由机型配置统一管理：

```bash
python -m arx_d_can.examples.dual_arm.example_07_send_position_mit \
  --left "0,0,0,90,0,0,0" \
  --right "0,0,0,90,0,0,0" \
  --velocity 60
```

MIT 示例同样使用统一的实际速度（命令行单位为度/秒），Kp/Kd 读取机型 YAML，
目标速度和前馈力矩为零。普通 MIT/PV 调用都是非阻塞的最新值控制。

普通 `conda run -n ...` 会缓存子进程输出，并可能在 Ctrl+C 时由 Conda 自己截获
中断，使 Python 的 `finally` 清理逻辑没有机会完成。运动控制不能依赖这种运行方式。

- 单次双通道批量发送失败或 streaming 命令超时会进入 `SAFE_HOLD`；
- 反馈缺失或过期只更新健康诊断，不改变 Runtime 运行状态；
- 安全保持失败、设备掉线、非法反馈、电机故障或意外失能会锁存 `FAULT`；夹爪
  故障默认执行 motor 产品 profile 内置的保持策略；
- `read_state()` 和 `read_cached_state()` 都读取 Runtime 后台持续刷新的完整缓存；
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
