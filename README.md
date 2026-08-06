# ARX-D-CAN Python SDK

独立的 ARX-D-CAN Python SDK，可通过 USB2CAN 串口或 Linux SocketCAN 控制
Damiao 关节电机和可选夹爪。默认机型包含 6 个机械臂关节；默认连接为
`dm-serial`、`/dev/ttyACM0`、波特率 `1000000`，控制模式为 `POS_VEL`。

## 安装

```bash
cd Articore-SDK
python -m pip install .
```

安装时会自动使用 `motor-drive-layer==0.5.6` 作为底层电机通信 SDK。该版本包含
dm-serial 批量反馈加固，以及 SocketCAN、SocketCAN-FD 和 dm-serial 后端。

运动学、动力学和末端控制需要 Pinocchio：

```bash
python -m pip install ".[dynamics]"
```

默认 URDF 已打进 wheel，不依赖开发者电脑上的绝对路径。

## USB2CAN 与 SocketCAN

SDK 的 `transport` 可选 `dm-serial`、`socketcan`、`socketcanfd` 或 `auto`：

| transport | channel/port 示例 | 用途 | `baud` 的含义 |
|---|---|---|---|
| `dm-serial` | `/dev/ttyACM0` | 达妙串口 USB2CAN | 串口波特率 |
| `socketcan` | `can0` | Linux 经典 CAN，例如 MCP2515 | 忽略；CAN 比特率由 Linux 配置 |
| `socketcanfd` | `can0` | Linux CAN-FD 控制器 | 忽略；CAN-FD 参数由 Linux 配置 |
| `auto` | 任一 | 兼容旧配置 | `/dev/tty*` 推断为 dm-serial，其他名称推断为 SocketCAN |

命令行中的 `--port` 和 `--channel` 是同一个参数的两个名字。Python API 同时接受
`channel=` 和为了兼容旧代码保留的 `port=`，即 SocketCAN 可写作
`ArxDCanArm(transport="socketcan", channel="can0")`。两个别名不能同时指定不同值。
建议新部署显式指定 `transport`，不要依赖自动推断。

RK3588、MCP2515、经典 CAN 1 Mbps 的完整配置、扫描、读状态、控制和排错说明见
[SocketCAN 使用说明](docs/socketcan.md)。最常用的扫描命令是：

```bash
python -m arx_d_can.examples.example_01_scan_ids \
  --transport socketcan \
  --channel can0 \
  --feedback-base 0x200
```

这里 `--feedback-base 0x200` 不是固定值。扫描器按
`feedback_id = feedback_base + (motor_id & 0x0F)` 注册临时电机；例如电机 ID 为
`6`、实际反馈 ID 为 `0x206` 时，必须传 `0x200`。默认 `0x10` 适用于反馈 ID
`0x11`～`0x1F` 的现有机型。

## 使用顺序

```bash
python -m arx_d_can.examples.example_01_scan_ids --port /dev/ttyACM0
python -m arx_d_can.examples.example_02_read_state --port /dev/ttyACM0
python -m arx_d_can.examples.example_03_clear_faults --port /dev/ttyACM0
python -m arx_d_can.examples.example_04_send_position \
  --positions "0,-20,-20,0,0,0" \
  --velocity-limits "120,120,120,90,90,90" --port /dev/ttyACM0
python -m arx_d_can.examples.example_04_send_position \
  --positions "0,-20,-20,0,0,0" --mode mit \
  --velocities "0,0,0,0,0,0" \
  --torques "0,0,0,0,0,0" --port /dev/ttyACM0
python -m arx_d_can.examples.example_05_gripper_open_close --port /dev/ttyACM0
python -m arx_d_can.examples.example_06_benchmark_read_rate \
  --port /dev/ttyACM0 --target-hz 500 --seconds 5
python -m arx_d_can.examples.example_07_send_joint_trajectory \
  "0,-60,-60,0,0,0" --port /dev/ttyACM0 --return-zero
python -m arx_d_can.examples.example_08_return_zero --port /dev/ttyACM0
python -m arx_d_can.examples.example_09_diagnose_status --port /dev/ttyACM0
python -m arx_d_can.examples.example_11_record_and_replay_trajectory \
  record trajectory.json --seconds 10 --hz 100 --port /dev/ttyACM0
python -m arx_d_can.examples.example_11_record_and_replay_trajectory \
  replay trajectory.json --port /dev/ttyACM0
python -m arx_d_can.examples.example_12_gravity_compensation \
  --port /dev/ttyACM0 --seconds 10
```

`example_04_send_position.py` 直接发送目标，不做插值或回零，并在发送后默认持续
刷新目标。它通过 `--mode pv`（默认）使用 POS_VEL 位置速度模式，也可通过
`--mode mit` 使用 MIT 模式；MIT 的 `kp/kd` 和 PV 的环路参数均读取
所选机型的硬件 YAML。MIT 还可用 `--torques` 传入每个关节的
前馈力矩，单位为 N·m，并用 `--velocities` 传入每个关节的目标速度；PV 用
`--velocity-limits` 覆盖配置
中的各关节最大速度。速度命令行参数单位均为 deg/s。MIT 的速度和力矩未提供时默认
为全零，PV 未提供限速时使用 YAML 中各关节的 `vlim`。MIT 目标速度是阻尼项输入，
不是最大速度限制；需要严格控制运动速度时应使用示例 07 生成插值轨迹。使用
`Ctrl+C` 会失能全部电机，停止前必须托住机械臂；
只有显式传入正数 `--hold-seconds` 时才会定时退出。平滑轨迹使用示例 07，直接回零
使用示例 08。

## 安全机制

- 任一关节发送失败、明确电机故障或连续 3 次反馈失败时，SDK 锁存故障并尝试
  整臂失能。
- 使能后有 2 秒启动宽限；第一帧成功命令之后，超过 0.25 秒没有新命令，软件
  看门狗读取实际关节位置并以 100 Hz 进入 `SAFE_HOLD`，保持手臂和夹爪当前位置。
- `SAFE_HOLD` 期间如果保持指令发送失败，会升级为硬故障并尝试整臂失能。
- 故障不会自动恢复。确认硬件和空间安全后调用 `recover()`；低层 API 也可以依次
  调用 `clear_fault()`、`configure()`、`enable()`。
- `close()` 总是停止看门狗、尝试失能所有电机并关闭总线。

看门狗参数位于 `arx_d_can/config/arx_d_can_dm.yaml` 的 `safety`。它是进程内
软件看门狗，
能处理控制线程卡住或上游停止发命令；它不能覆盖整机掉电、Python 进程被强制
杀死或 USB2CAN 硬件失效。`SAFE_HOLD` 也不是安全认证功能，生产设备仍需要物理
急停、电机侧通信超时，以及垂直负载场景需要的机械制动或防坠机构。

## Python API

```python
import time
from arx_d_can import ArxDCanArm

target = [0.0, -1.047, -1.047, 0.0, 0.0, 0.0]
arm = ArxDCanArm(port="/dev/ttyACM0")
try:
    arm.connect()
    arm.configure()
    arm.enable()
    while True:
        arm.send_joint_positions(target)
        time.sleep(0.01)
finally:
    arm.close()
```

### 重要：MIT 支持显式发送 Kp=0、Kd=0

`send_joint_positions()` 的 `mit_kp`、`mit_kd` 支持标量 `0` 和包含零的关节数组。
传入标量 `0` 时，SDK 会把它展开成全部机械臂关节的零增益，并原样写入当前 MIT
控制帧；`0` 不会被当成“未设置”。因此，下面两种写法等价：

```python
# 标量 0：应用到全部关节（推荐）
arm.send_joint_positions(
    positions,
    torques=torques,
    mit_kp=0,
    mit_kd=0,
    mode="mit",
)

# 也可以逐关节明确传入 0；数组长度必须等于当前机型的机械臂关节数
arm.send_joint_positions(
    positions,
    torques=torques,
    mit_kp=[0.0] * len(arm.joint_names),
    mit_kd=[0.0] * len(arm.joint_names),
    mode="mit",
)
```

不要省略 `mit_kp` 或 `mit_kd` 来表示零增益。省略参数（默认值为 `None`）表示使用
机型 YAML 中配置的默认增益。也就是说，只传 `torques` 仍然是“位置阻抗控制 +
前馈力矩”，不是纯力矩控制：

```python
arm.send_joint_positions(
    positions,
    velocities=velocities,
    torques=torques,
    mit_kp=[20.0, 20.0, 20.0, 5.0, 5.0, 5.0],
    mit_kd=[2.0, 2.0, 2.0, 0.5, 0.5, 0.5],
    mode="mit",
)
```

`positions` 在 MIT 数据帧中仍是必填字段，但当 `mit_kp=0` 时，位置误差不再产生
控制力矩；同理，当 `mit_kd=0` 时，目标速度误差不再产生控制力矩。纯力矩输出由
`torques` 决定。SDK 提供 `GravityCompensationMode` 来完成重力计算、安全检查和
零增益发送：

```python
from arx_d_can import ArxDCanArm, GravityCompensationMode

arm = ArxDCanArm(
    model="yunyi_v1_0_right",
    port="/dev/ttyACM0",
    control_mode="mit",
)
gravity = GravityCompensationMode(arm, damping=0.0)
try:
    gravity.start()
    gravity.run(seconds=10.0)
finally:
    gravity.shutdown()
```

该模式会从当前位置保持开始，逐帧把 Kp/Kd 降到目标值并把补偿力矩升到目标值；
退出时执行反向过渡，然后失能。默认 `damping=0`，即活动阶段明确发送
`mit_kp=0、mit_kd=0`；需要速度阻尼时可设置较小的非零 `damping`。模式还会检查
力矩、速度、URDF 关节限位和 SDK 故障状态。

纯力矩模式没有位置保持能力，必须持续发送经过限幅和安全检查的力矩。如果希望保留
速度阻尼，可以使用 `mit_kp=0` 和较小的非零 `mit_kd`。命令超时后，SDK 看门狗会在
最后一次成功发送的位置恢复 YAML 默认 MIT 增益进行安全保持；这个回退不是继续发送
零增益重力补偿。生产设备还必须具备物理急停、电机侧保护和可靠的防坠措施。

`example_04_send_position.py` 的命令行参数目前不提供 Kp/Kd 覆盖选项，因此即使传入
`--torques`，它仍使用 YAML 默认 Kp/Kd。需要发送 `Kp=0、Kd=0` 时，请使用上面的
Python API，或直接运行示例 12：

```bash
python -m arx_d_can.examples.example_12_gravity_compensation \
  --arm-model yunyi_v1_0_right \
  --port /dev/ttyACM0 \
  --seconds 10
```

## 多机型配置

SDK 不再在代码中假定机械臂必须是 6 轴。关节数量、顺序、电机 ID、反馈 ID、
电机型号、MIT/PV 参数、夹爪和 URDF 都来自一个机型 YAML；高层 SDK 与低层驱动
共用同一次解析结果，避免两层加载到不同配置。

内置机型在 `arx_d_can/config/models.yaml` 注册。以后增加一种随 SDK 发布的机械臂：

1. 复制 `arx_d_can/config/arx_d_can_dm.yaml`，创建该机型自己的 YAML，并修改
   `groups.arm.joints`、`groups.gripper`、各电机参数和 URDF。
2. 在 `models.yaml` 的 `models` 中增加 `机型名: YAML文件名`。
3. 通过 `ArxDCanArm(model="机型名")` 或示例参数 `--arm-model 机型名` 选择。

```python
from arx_d_can import ArxDCanArm, available_models

print(available_models())
arm = ArxDCanArm(model="arx_d_can", port="/dev/ttyACM0")
```

只是本地测试新机械臂时，不必修改注册表，直接传外部 YAML：

```python
arm = ArxDCanArm(config_path="/path/to/my_arm.yaml")
```

对应的示例命令为 `--config-path /path/to/my_arm.yaml`。`--arm-model` 与
`--config-path` 互斥；没有指定时使用 `models.yaml` 的 `default_model`。
若某个电机的正方向与机械臂坐标相反，在该关节配置中设置 `direction: -1`；
SDK 会同时反转位置、速度和力矩的指令及反馈，其他关节省略该字段即可。
若实际电机的 MIT 力矩映射范围与底层型号默认值不同，设置
`torque_range`；SDK 会同时换算 MIT 前馈力矩和反馈力矩。

### Corina V2 双腿

Corina V2 作为一个 12 关节机型注册，使用同一条 CAN 总线。右腿 ESC ID 为
`0x01～0x06`，左腿为 `0x07～0x0C`。所有 ESC ID 均保持在达妙状态帧可唯一表达的
4 位范围内，避免 `0x11/0x12` 与 `0x01/0x02` 的低 4 位冲突。反馈/MST ID 统一为
`ESC_ID + 0x20`：右腿是 `0x21～0x26`，左腿是 `0x27～0x2C`。

```python
from arx_d_can import ArxDCanArm

robot = ArxDCanArm(model="corina_v2", port="/dev/ttyACM0")
print(robot.joint_names)
```

控制参数暂时沿用 SDK 中 4340P/4310 的保守默认值。实机方向标定为右腿关节1～4
以及左腿关节2～3使用 `direction: -1`，其余关节为正向。`groups.arm.joints` 包含
双腿全部 12 个关节。为匹配 Pinocchio 对该分支 URDF 的模型顺序，所有位置、速度
和力矩数组按先左腿 6 轴、后右腿 6 轴的顺序传入。

Corina 的关节5/6按 URDF 定义分别作为脚端 Pitch/Roll 使用。读取状态、位置轨迹、
MIT 速度和前馈力矩均使用这两个关节坐标；用户不需要配置额外的执行器换算。

### Yunyi V1.0 双臂

Yunyi V1.0 只保留一份完整双臂模型：
`arx_d_can/models/yunyi_v1_0.urdf`。左右机型配置共同引用该文件；运动学控制器按照
`groups.arm.joints` 构建本侧 7 轴 reduced model，因此每个 USB2CAN 仍只控制一侧，
而不会复制或裁剪 URDF：

| 单臂电机 | 型号 | 右臂 CAN/反馈 ID | 左臂 CAN/反馈 ID |
|---|---|---|---|
| joint1～joint2 | 8009 | 0x01～0x02 / 0x11～0x12 | 0x09～0x0A / 0x19～0x1A |
| joint3～joint4 | 4340P | 0x03～0x04 / 0x13～0x14 | 0x0B～0x0C / 0x1B～0x1C |
| joint5～joint7 | 4310 | 0x05～0x07 / 0x15～0x17 | 0x0D～0x0F / 0x1D～0x1F |
| gripper（第 8 个电机） | 4310 | 0x08 / 0x18 | 0x01 / 0x11 |

左右臂使用独立 USB2CAN。当前左臂使用 `/dev/ttyACM0`，右臂默认使用
`/dev/ttyACM1`；Linux 设备号发生变化时显式覆盖
`port`：

```python
right_arm = ArxDCanArm(
    model="yunyi_v1_0_right",
    port="/dev/ttyACM1",
    enable_gripper=True,
)
left_arm = ArxDCanArm(
    model="yunyi_v1_0_left",
    port="/dev/ttyACM0",
    enable_gripper=True,
)
```

也可以通过所有编号示例单独操作一侧，例如：

```bash
python -m arx_d_can.examples.example_02_read_state \
  --arm-model yunyi_v1_0_left \
  --port /dev/ttyACM0
```

示例 02 默认以 10 Hz 持续打印，按 `Ctrl+C` 停止；需要只读取一次时增加
`--once`。

当前配置将第 8 个 4310 作为一个夹爪电机，机械联动 URDF 中的两根手指。MIT/PV
初始增益沿用现有 ARX 机型的保守参数，不视为 Yunyi 实机最终标定值；首次使能前
应托稳单臂、卸载负载，并逐关节验证方向、零点和增益。左臂关节 `0x09～0x0F`
和夹爪 `0x01/0x11` 已在 `/dev/ttyACM0` 实机确认。

## 维护工具

维护工具与普通示例分开。调零命令会先确认机械臂静止，再把当前位置逐关节写为
零位。每个电机写入后必须连续收到 3 帧新反馈，且状态正常、位置接近零、速度接近
零，才判定成功：

```bash
python -m arx_d_can.service_tools.zero_current_position --port /dev/ttyACM0
```

相同的安全调零流程也提供了编号示例：

```bash
python -m arx_d_can.examples.example_10_set_zero_current_position \
  --port /dev/ttyACM0
```

默认只调所选机型的手臂关节；夹爪另加 `--include-gripper`。其他维护工具：

```bash
python -m arx_d_can.service_tools.change_damiao_id --port /dev/ttyACM0
python -m arx_d_can.service_tools.joint_load_probe \
  --port /dev/ttyACM0 --joint 4 --amplitude-deg 10 --csv /tmp/joint4.csv
```

## 配置

默认机型列表位于 `arx_d_can/config/models.yaml`；每种机械臂的硬件 ID、反馈 ID、
控制增益、关节分组、夹爪映射和安全参数位于各自的硬件 YAML。默认机型使用
`arx_d_can/config/arx_d_can_dm.yaml`。VR/ROS 上层已经负责工作空间和 URDF
关节限位；SDK 安全层负责通信故障、命令超时保持和退出失能。

## 开发验证

源码采用根目录 `arx_d_can/` 包布局。安装开发依赖后可直接运行测试和构建：

```bash
python -m pip install ".[dev]"
python -m pytest --import-mode=importlib --rootdir=tests tests
python -m pip wheel --no-deps . --wheel-dir dist
```
