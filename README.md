# Articore-SDK 1.0

Yunyi V1.0 双臂 Python SDK。1.0 起，Python 只通过 Cyclone DDS/IP 调用部署在
RK3588 上的 `articore-runtime-service`，不再加载本地 Runtime `.so`，也不再直接
访问 SocketCAN。

电机协议、双 CAN-FD、500 Hz 控制、安全状态机、产品模型、IK、轨迹、夹爪和重力
补偿全部留在 RK3588 C++ Runtime。Python 是薄客户端：参数校验、DDS 请求/应答、
控制租约、最新流式目标发送和状态缓存。

## 安装

```bash
conda activate at
cd ~/Articore-SDK
pip install -e .
```

SDK 1.0.3 固定依赖 `cyclonedds==11.0.1`，对应 RK3588 Runtime 1.2.0 和 DDS v1.2
协议。`RobotState` 是 DDS `@final` 类型，v1.2 新增夹爪状态字段后不再兼容
Runtime v1.0/v1.1；连接旧服务会明确返回 `VERSION_MISMATCH`。原来的 `motor-drive-layer`、ctypes
`_motor_abi` 和本地 native wheel 已从
1.0 包中删除。

## RK3588 前提

先在板端安装并启动 `articore-runtime-service`：

```bash
sudo apt install /tmp/articore-runtime-service_1.0.3_arm64.deb
sudo systemctl enable --now articore-can.service
sudo systemctl enable --now articore-runtime.service
systemctl status articore-runtime.service --no-pager
```

客户端和板端必须使用相同的 `robot_id`、DDS Domain ID，并且所选网卡 IP 可达。默认
配置是 `robot_id="yunyi-001"`、`domain_id=0`。服务启动和 DDS 连接都不会自动使能
电机。

## 最小用法

```python
from arx_d_can import ArxDCanDualArm

robot = ArxDCanDualArm(
    robot_id="yunyi-001",
    robot_ip="192.168.1.185",
    domain_id=0,
    control_mode="pv",
)

try:
    robot.connect()  # 获取租约并保活；READY 时配置模式，FAULT 时进入维护会话
    robot.enable()
    robot.set_joint_pv(
        left=[0.0] * 7,
        right=[0.0] * 7,
        velocity=50,
    )
    state = robot.read_state()
    print(state.left.positions, state.right.positions)
    print(robot.get_health())
finally:
    robot.disable()
    robot.disconnect()  # 释放租约和本客户端 DDS 资源，不关闭板端 Runtime
```

构造参数：

- `robot_id`：目标机器人 DDS key，默认 `yunyi-001`。
- `robot_ip`：RK3588 单播发现 Peer；当前机械臂填写 `192.168.1.185`。配置后仍保留
  multicast，因此有线直连和普通局域网发现可以同时工作。
- `domain_id`：Cyclone DDS Domain，默认 `0`。
- `client_id`：控制客户端标识；省略时自动生成。
- `security_identity`：协议预留身份字符串；可信网络首版默认留空，不等同于认证。
- `network_interfaces`：显式使用的一个或多个本机网卡，顺序靠前者优先。省略时尊重
  Cyclone DDS 默认配置或 `CYCLONEDDS_URI`。
- `control_mode="mit" | "pv"`。
- `request_timeout` 与 `discovery_timeout`：请求及发现超时秒数。

Runtime v1.1 在每次服务启动时分别扫描左右 ID8 末端，允许仅左侧、仅右侧、双侧或
不安装夹爪。Python 不配置物理拓扑，连接后通过 `robot.hardware_topology`、
`robot.left_has_gripper` 和 `robot.right_has_gripper` 读取扫描结果。旧的
`with_grippers` 构造参数仅为源码兼容而保留，不再覆盖板端扫描结果。更换末端应先
失能并重启 Runtime，新的 Runtime 进程会重新扫描，但不会自动 enable。

Runtime v1.2 还会在 500 Hz `RobotState` 中发布左右夹爪开合度。SDK 只有在对应侧
`gripper_available` 和 `gripper_feedback_valid` 同时为真且开合度为有限值时，才在
`read_state()` 的 `state.left.gripper` / `state.right.gripper` 中返回该侧夹爪状态；
反馈过期时返回 `None`，物理安装情况仍以 topology 属性为准。

现有示例不传 `robot_ip`，可以先设置环境变量，让全部示例都直连这台机械臂：

```bash
export ARTICORE_ROBOT_IP=192.168.1.185
```

有线直连时，本机有线网卡还必须配置为同网段地址（例如 `192.168.1.100/24`），但不能
占用机械臂的 `192.168.1.185`。

## 控制和租约语义

整台机器人同一时刻只有一个控制租约。SDK 在 `connect()` 成功后以 20 Hz 保活；租约
丢失后，板端会撤销流式目标、取消有限运动并走 Runtime 安全停止路径。重新连接只会
重新获取租约，不会自动使能电机。

Runtime 已处于 `FAULT` 时，`connect()` 会保留租约和 heartbeat，但不会先发送
`CONFIGURE_MODE`。此维护会话允许应用显式调用 `clear_motor_faults()`；清错成功后
SDK 才配置构造时要求的 PV/MIT 模式。连接、清错和模式配置均不会自动 enable，也
不会发送运动目标。急停锁存不能由普通清错解除，必须按 Runtime 的恢复流程处理。

纯维护工具应显式调用 `connect(maintenance=True)`。这种连接即使 Runtime 当前为
`READY` 也不会配置模式，而且 `clear_motor_faults()` 成功后仍保持未配置、失能状态；
需要进入控制流程时，先断开维护连接，再使用普通 `connect()` 建立业务会话。这样可
避免电机尚未静止时，清错工具在连接阶段被 `CONFIGURE_MODE` 拦截。

- `set_joint_pv()`、`set_joint_mit()` 和 `set_joint_mit_fast()` 发送最新完整 14 轴目标。
  流式 Topic 为 Best Effort、KeepLast(1)、20 ms Lifespan；丢包时不补发旧目标。
- 普通 PV 的 500 Hz 在线参考、速度/加速度包络和到位判断全部在 Runtime 内部。
  Python 不插值、不生成 P 步进，也不重采样。
- `move_pose()`、`move_linear()`、`move_circular()` 是 Runtime 内部执行的有限笛卡尔
  运动，与普通流式目标严格区分。
- DDS v1 只支持整机原子 `enable()` / `disable()`；不支持 `motors=[...]` 选择性电源
  操作。
- DDS v1 的 Linear 请求只携带单个终点；不支持 Python `poses=[...]` 显式多段路径。

标准 MIT 示例：

```python
robot.set_joint_mit(
    left_positions=left_q,
    right_positions=right_q,
    left_velocities=left_dq,
    right_velocities=right_dq,
    kp=20.0,
    kd=1.0,
    left_feedforward_torques=[0.0] * 7,
    right_feedforward_torques=[0.0] * 7,
)
```

快速 MIT 面向遥操和高频连续小步目标：

```python
robot.set_joint_mit_fast(left=left_q, right=right_q, velocity=10)
```

## 状态与产品能力

- `read_state()` 返回 RK3588 以 500 Hz 发布的最新关节位置、速度、力矩、温度、使能
  位、时间戳、序号和 `motion_arrived`；读取 Python 缓存，不向 CAN 发请求。
- `get_fps()` 根据最近 DDS state 样本到达时间计算接收频率，不再表示本地 CAN 帧率。
- `get_health()` 返回服务健康、安全保持、失能确认、发送/反馈连续失败计数和故障原因。
- `get_joint_limits()`、`get_pose()`、`solve_ik()`、TCP offset、速度/加速度配置、夹爪、
  重力补偿、双臂跟随、急停、恢复、清错和置零均通过可靠 DDS 控制请求调用 Runtime。
- 请求失败抛出 `RuntimeCallError`；其 `code` 是稳定的 `RuntimeErrorCode`，包括
  `NO_LEASE`、`STALE_SEQUENCE`、`TIMEOUT`、`TRANSPORT_ERROR` 和
  `VERSION_MISMATCH` 等协议错误。
- 当前 DDS v1 的 `RobotState` 不携带夹爪反馈明细，因此 `read_state().left.gripper` 和
  `right.gripper` 为 `None`；夹爪命令仍由 `set_grippers()` 完整支持。

所有姿态为 `[x, y, z, roll, pitch, yaw]`，位置单位米、姿态单位弧度。所有关节数组
固定为左臂 J1..J7 后接右臂 J1..J7。

## 常用命令

```bash
conda activate at
cd ~/Articore-SDK

pytest -q

python -m arx_d_can.examples.diagnostics.example_01_read_state \
  --mode continuous --display-hz 10

python -m arx_d_can.examples.diagnostics.example_02_benchmark_read_rate \
  --seconds 15 --hz 500

python -m arx_d_can.examples.diagnostics.example_03_read_health
```

完整示例见 `arx_d_can/examples/README.md`。运行任何会运动的示例前，必须完成板端 CAN
反馈与健康检查、确认工作空间安全，并由现场人员单独授权使能。

## 架构边界

```text
Python business code
  └─ ArxDCanDualArm
       └─ Cyclone DDS v1 / IP
            └─ RK3588 articore-runtime-service
                 └─ dds → runtime (500 Hz/safety/model/trajectory) → motor/SocketCAN-FD
```

SDK 不包含 Motor、Controller、寄存器、CAN 扫描、Pinocchio 控制实现或任何本地
Runtime ABI。URDF 仍随 SDK 分发，仅用于展示、仿真和外部工具。
