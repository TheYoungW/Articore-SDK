# ARX-D-CAN Python SDK

独立的 ARX-D-CAN Python SDK，通过 USB2CAN 串口控制 Damiao 关节电机和
可选夹爪。默认机型包含 6 个机械臂关节；默认串口为 `/dev/ttyACM0`，波特率为 `1000000`，控制模式为
`POS_VEL`。

## 安装

```bash
cd Articore-SDK
python -m pip install .
```

安装时会自动使用 `motor-drive-layer==0.5.1` 作为底层电机通信 SDK。

运动学、动力学和末端控制需要 Pinocchio：

```bash
python -m pip install ".[dynamics]"
```

默认 URDF 已打进 wheel，不依赖开发者电脑上的绝对路径。

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

### MIT 逐帧增益与纯力矩控制

`send_joint_positions(..., torques=...)` 中的 `torques` 是 MIT 前馈力矩。默认仍会
使用机型 YAML 中的 Kp/Kd，因此不是纯力矩控制。可通过 `mit_kp`、`mit_kd` 对当前
一帧覆盖各关节增益；不传入时继续使用 YAML 默认值：

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

交接阶段可逐帧将 `mit_kp`、`mit_kd` 从 YAML 增益平滑降到零。进入纯力矩阶段后，
直接把两个增益设为标量 `0`；标量会自动应用到全部关节：

```python
arm.send_joint_positions(
    positions,
    torques=torques,
    mit_kp=0,
    mit_kd=0,
    mode="mit",
)
```

纯力矩模式没有位置保持能力，必须持续发送经过限幅和安全检查的力矩。命令超时后，
SDK 看门狗仍会尝试读取当前位置并恢复安全保持；生产设备还必须具备物理急停和
电机侧保护。

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
| gripper（第 8 个电机） | 4310 | 0x08 / 0x18 | 0x10 / 0x20 |

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

当前配置将第 8 个 4310 作为一个夹爪电机，机械联动 URDF 中的两根手指。MIT/PV
初始增益沿用现有 ARX 机型的保守参数，不视为 Yunyi 实机最终标定值；首次使能前
应托稳单臂、卸载负载，并逐关节验证方向、零点和增益。左臂 `0x09～0x0F` 已在
`/dev/ttyACM0` 实机确认，左臂第 1、4 关节已配置为反向；预留夹爪
`0x10/0x20` 当前未收到反馈。

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
