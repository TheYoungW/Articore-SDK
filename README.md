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

### SocketCAN 配置与使用

SocketCAN 适用于 RK3588 上的 MCP2515、板载 CAN 控制器，以及由其他驱动暴露出的
`can0`/`can1`。MCP2515 是经典 CAN 控制器，应使用 `transport: socketcan`，不要
使用 `socketcanfd`。

安装或更新项目后，可确认底层通信库版本及 Python 环境：

```bash
python -m pip install -e .
python -m pip show motor-drive-layer
which python
python -m pip --version
```

不要混用系统 Python 与 conda Python；`python` 和 `python -m pip` 应属于同一环境。

以下命令把 `can0` 设置为经典 CAN 1 Mbps：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0
```

正常通信时，接口应为 `UP`，CAN 状态应为 `ERROR-ACTIVE`。`ERROR-PASSIVE`、
`BUS-OFF` 或持续增长的 error counter 通常表示终端电阻、波特率、CAN_H/CAN_L、
共地或供电存在问题。SocketCAN 模式下，YAML 的 `baud` 和命令行 `--baud` 不配置
CAN 总线速率；总线速率只由 `ip link` 配置。可先用 `can-utils` 验证原始总线：

```bash
candump can0
ip -details -statistics link show can0
```

#### 扫描电机 ID

扫描不会使能电机或发送运动目标：

```bash
python -m arx_d_can.examples.example_01_scan_ids \
  --transport socketcan \
  --channel can0 \
  --model 4340P \
  --start-id 1 \
  --end-id 16 \
  --feedback-base 0x200
```

`--port can0` 与 `--channel can0` 等价。`--feedback-base 0x200` 不是固定值；
扫描器按以下公式计算每个候选电机的预期反馈 ID：

```text
expected_feedback_id = feedback_base + (motor_id & 0x0F)
```

例如，电机 ID `6`、反馈 ID `0x206` 必须使用 `--feedback-base 0x200`；电机 ID
`1`、反馈 ID `0x11` 使用默认值 `0x10`。若电机的 MST_ID 不符合这一组连续映射，
扫描器无法仅根据 ESC_ID 推导它；必须先获知实际反馈 ID，再写入机型 YAML，或用
底层单电机命令明确指定 `--motor-id` 和 `--feedback-id`。

扫描结果还会校验以下内容，避免把其他电机的反馈帧误报为当前候选 ID：

- 实际 `arbitration_id` 与候选电机的预期反馈 ID 完全一致；
- 状态码位于 Damiao 的有效范围；
- 位置、速度、力矩和温度均为有限数；
- 温度不是 `255°C` 等无效哨兵值；
- 位置、速度和力矩不会同时处于所选电机型号的编码极值。

若已知电机没有被扫描到，先检查 `--model`、`--feedback-base` 和 Linux CAN 错误
计数，不要盲目扩大 ID 范围。

#### YAML 与命令行配置

自定义机型配置中应明确写出后端和通道：

```yaml
name: RK3588 Arm
transport: socketcan
channel: can0
baud: 1000000  # 仅为兼容字段；SocketCAN 模式下不设置总线比特率
rate: 500
```

内置机型目前明确配置为 `transport: dm-serial`。使用同一机型但临时改走板载 CAN
时，不必编辑内置 YAML，可直接通过命令行覆盖：

```bash
python -m arx_d_can.examples.example_02_read_state \
  --arm-model yunyi_v1_0_right \
  --transport socketcan \
  --channel can0
```

#### Python API

读状态和运动控制使用同一组显式连接参数：

```python
import time
from arx_d_can import ArxDCanArm

arm = ArxDCanArm(
    model="yunyi_v1_0_right",
    transport="socketcan",
    channel="can0",
)
try:
    arm.connect()
    print(arm.read_state())

    arm.configure()
    arm.enable()
    target = arm.read_state().positions
    while True:
        arm.send_joint_positions(target)
        time.sleep(0.01)
finally:
    arm.close()
```

上例的运动部分会使能整臂并持续保持当前位置，只能在机械臂处于安全环境、有人托稳
且具备急停时运行。只验证通信时，到 `print(arm.read_state())` 为止，不要调用
`configure()` 或 `enable()`。低层 API 也支持同样的连接配置：

```python
from arx_d_can import ArxDCan

arm = ArxDCan(model="yunyi_v1_0_right", transport="socketcan", channel="can0")
arm.connect()
```

#### SocketCAN 常见问题

- `No such device`：Linux 没有名为 `can0` 的接口，先检查 `ip link` 和设备树驱动。
- 扫描结果为 `none`：优先检查接口是否 `UP`、1 Mbps 是否一致、反馈 ID 基址和电机
  型号是否正确。
- 扫描到错误 ID：SDK 会校验实际仲裁 ID；若仍出现，保存完整扫描输出和
  `candump can0`，确认总线上是否有重复反馈 ID。
- `BUS-OFF`：先修复物理层和波特率，再重新拉起接口；应用层重试不能修复总线错误。
- `/dev/ttyACM0` 是 dm-serial 设备，不是 SocketCAN；应使用
  `--transport dm-serial --channel /dev/ttyACM0`。
- `can0` 是网络接口，不是文件路径；应使用
  `--transport socketcan --channel can0`，不需要也不应修改 `/dev/ttyACM0` 权限。

## 使用顺序

```bash
python -m arx_d_can.examples.example_01_scan_ids --port /dev/ttyACM0
python -m arx_d_can.examples.example_02_read_state --port /dev/ttyACM0
python -m arx_d_can.examples.example_03_clear_faults --port /dev/ttyACM0
python -m arx_d_can.examples.example_04_send_position \
  --positions "0,-20,-20,0,0,0" --mode pv --port /dev/ttyACM0
python -m arx_d_can.examples.example_05_gripper_open_close --port /dev/ttyACM0
python -m arx_d_can.examples.example_06_benchmark_read_rate \
  --port /dev/ttyACM0 --hz 500 --seconds 5
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

示例只展示高层接口。命令刷新、轨迹插值、夹爪防堵转、反馈检查和退出失能均由 SDK
内部完成。示例 04 用于持续保持一个目标，示例 07 用于平滑运动，示例 08 用于平滑
返回零位。控制模式目前只支持 PV 和 MIT。

## 安全机制

- 任一关节发送失败、明确电机故障或连续 3 次反馈失败时，SDK 锁存故障并尝试
  整臂失能。
- 使能后有 2 秒启动宽限；第一帧成功命令之后，超过 0.25 秒没有新命令，软件
  看门狗读取实际关节位置并以 100 Hz 进入 `SAFE_HOLD`，保持手臂和夹爪当前位置。
- `SAFE_HOLD` 期间一般发送失败会记录原因并按保持频率继续重试；耦合关节反馈过期
  属于不可继续使用旧状态的硬故障，会尝试整臂失能。
- 故障不会自动恢复。确认硬件和空间安全后调用 `recover()`；低层 API 也可以依次
  调用 `clear_fault()`、`configure()`、`enable()`。
- `close()` 总是停止看门狗、尝试失能所有电机并关闭总线。

看门狗参数位于 `arx_d_can/config/arx_d_can_dm.yaml` 的 `safety`。它是进程内
软件看门狗，
能处理控制线程卡住或上游停止发命令；它不能覆盖整机掉电、Python 进程被强制
杀死或 USB2CAN 硬件失效。`SAFE_HOLD` 也不是安全认证功能，生产设备仍需要物理
急停、电机侧通信超时，以及垂直负载场景需要的机械制动或防坠机构。

### 通讯异常和健康状态

SDK 对外提供稳定的异常层级，不要求应用依赖底层 `motor-drive-layer.CallError`：

- `CommunicationError`：所有传输和反馈通讯异常的基类；
- `TransportError`：串口或 CAN 打开、读取、写入失败；
- `FeedbackTimeoutError`：完整新反馈超时；
- `IncompleteFeedbackError`：缺少一个或多个必需电机的反馈；
- `StaleFeedbackError`：缓存反馈年龄超过安全阈值；
- `MotorFaultError`：电机明确返回故障状态码，不属于通讯故障；
- `CommandTimeoutError`：上层停止更新命令，不属于总线通讯故障。

这些运行期异常都继承 `RuntimeError`，已有的 `except RuntimeError` 仍然有效。
`read_state()` 始终请求新鲜、完整的反馈，第一次失败时就会抛出通信异常，不会把
历史缓存伪装成当前状态：

```python
from arx_d_can import CommunicationError

try:
    state = arm.read_state()
except CommunicationError as exc:
    print(exc.operation, exc.motor_names, exc.retryable)
```

只需要最近一次成功状态且不希望发送通信帧时，应显式读取缓存：

```python
state = arm.read_cached_state()
```

同时可以读取当前通讯健康快照：

```python
health = arm.communication_health
print(health.healthy)
print(health.consecutive_feedback_failures)
print(health.using_fallback_state)
print(health.last_fresh_feedback_age_s)
print(health.last_error)
```

一次成功的 `read_state()` 会清除连续失败计数和当前通讯错误；
`read_cached_state()` 不会把通讯失败计数清零。后台安全监控仍可在 SDK 内部短暂使用
最后状态进行安全保持，但不会通过公开的 `read_state()` 静默返回给用户。

## Python API

```python
from arx_d_can import ArxDCanArm

arm = ArxDCanArm(model="yunyi_v1_0_right", port="/dev/ttyACM0")
try:
    arm.connect()
    arm.configure()
    arm.enable()

    target = [0.0] * len(arm.joint_names)
    state = arm.move_joint_positions(target, seconds=6.0)
    print(state.arm.positions)
finally:
    arm.close()
```

常用接口保持简短：

```python
state = arm.read_state()                         # 新鲜完整反馈
state = arm.read_cached_state()                  # 最近一次成功反馈
arm.hold_joint_positions(target)                 # 持续保持目标
arm.move_joint_positions(target, seconds=6.0)    # 平滑运动
arm.move_gripper(1000)                           # 张开夹爪
arm.move_gripper(0)                              # 闭合夹爪
diagnostics = arm.read_motor_diagnostics()        # 只读诊断
```

### 高级 MIT 控制

普通位置控制使用机型 YAML 中的默认增益。只有实现阻抗控制或重力补偿时，才需要
直接使用 `send_joint_positions()` 的速度、力矩和 Kp/Kd 参数。显式传入
`mit_kp=0、mit_kd=0` 表示零增益；省略参数则使用机型默认值。

重力补偿已经封装为独立高层模式：

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

MIT 零增益没有位置保持能力，生产设备仍必须具备物理急停、电机侧保护和可靠的
防坠措施。

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
若实际电机的 MIT 协议映射范围与底层型号默认值不同，设置
`torque_range`；SDK 会同时换算 MIT 前馈力矩和反馈力矩。该字段不是安全限幅。
关节安全力矩使用 `effort_limit`，未显式配置时从 URDF `<limit effort>` 读取。
命令先按 `effort_limit` 裁剪，再按照 `torque_range` 或电机型号原生范围编码。

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

关节1～4控制参数沿用 SDK 中 4340P/4310 的保守默认值。实机方向标定为右腿关节1～4
以及左腿关节2～3使用 `direction: -1`，其余关节为正向。`groups.arm.joints` 包含
双腿全部 12 个关节。为匹配 Pinocchio 对该分支 URDF 的模型顺序，所有位置、速度
和力矩数组按先左腿 6 轴、后右腿 6 轴的顺序传入。

Corina 的关节5/6按 URDF 定义分别作为脚端 Pitch/Roll 使用。读取状态以及位置、
速度、Kp/Kd 和前馈力矩命令均使用这两个逻辑关节坐标；用户不需要配置额外的
执行器换算。

MIT 模式下，SDK 以 250 Hz 使用缓存的 A/B 电机反馈反算实际 Pitch/Roll，在逻辑
J5/J6 空间计算 PD 与前馈力矩，再通过耦合模型转换为 A/B 电机力矩。两个物理电机
的 MIT Kp/Kd 始终为零，不会把逻辑关节增益直接交给耦合电机。默认逻辑增益为
J5 `Kp=60, Kd=1.5`，J6 `Kp=30, Kd=0.8`。4310 协议仍使用原生
`±10 Nm` 映射，A/B 命令则按 URDF effort 独立限制在 `±7 Nm`，不会把 7 Nm
重新缩放成协议数值 10。可通过 `robot.coupled_torque_saturation` 读取最近一次
限幅状态。

每个内环周期都会读取 motor-drive-layer 的反馈 `update_count` 和 `age_ns`。
A/B 缓存反馈超过 Corina 当前配置的 50 ms 时立即停止虚拟 PD 并急停，不继续使用旧角度。
`robot.coupled_control_stats` 提供实际循环频率、周期超时次数、反馈停滞周期数和
最大反馈年龄，可用于真机确认单总线 12 电机的实际带宽。

策略循环可以按 200 Hz 更新完整逻辑 MIT 目标。命令看门狗会保留最后一帧的
位置、速度、Kp、Kd 和前馈力矩；进入安全保持后仍走上述虚拟关节控制链路，
不会回退到 YAML 增益作为 A/B 电机增益。

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
)
left_arm = ArxDCanArm(
    model="yunyi_v1_0_left",
    port="/dev/ttyACM0",
)
```

也可以通过所有编号示例单独操作一侧，例如：

```bash
python -m arx_d_can.examples.example_02_read_state \
  --arm-model yunyi_v1_0_left \
  --port /dev/ttyACM0
```

示例 02 读取并打印一次状态后自动退出。

当前配置将第 8 个 4310 作为一个夹爪电机，机械联动 URDF 中的两根手指。夹爪只使用
MIT 模式，默认 `Kp=4.0`、`Kd=0.5`；普通用户调用 `set_gripper(0..1000)` 即可，
也可以直接调用 `open_gripper()` 或 `close_gripper()`。Yunyi 默认启用内部防堵转：
检测到持续接触后降低保持刚度，持续过载时向张开方向回退；用户无需操作这套状态机。
机型配置中存在夹爪时，SDK 会默认连接并读取夹爪；更换末端的用户可显式传入
`enable_gripper=False`。反馈中的 `state.gripper.opening` 与控制接口使用相同的
`0～1000` 刻度；原始电机弧度保留在 `state.gripper.motor_position`。
机械臂关节的初始增益沿用现有 ARX 机型的保守参数，不视为 Yunyi 实机最终标定值；
首次使能前应托稳单臂、卸载负载，并逐关节验证方向、零点和增益。左臂关节
`0x09～0x0F` 和夹爪 `0x01/0x11` 已在 `/dev/ttyACM0` 实机确认。

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
