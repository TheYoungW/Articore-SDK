# ARX-D-CAN Python SDK

独立的 ARX-D-CAN Python SDK，通过 USB2CAN 串口控制 6 个 Damiao 关节电机和
可选夹爪。默认串口为 `/dev/ttyACM0`，波特率为 `1000000`，控制模式为
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
`src/arx_d_can/config/arx_d_can_dm.yaml`。MIT 还可用 `--torques` 传入六个关节的
前馈力矩，单位为 N·m，并用 `--velocities` 传入六个目标速度；PV 用
`--velocity-limits` 覆盖配置
中的六个最大速度。速度命令行参数单位均为 deg/s。MIT 的速度和力矩未提供时默认
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

看门狗参数位于 `src/arx_d_can/config/arx_d_can_dm.yaml` 的 `safety`。它是进程内
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

## 维护工具

维护工具与普通示例分开。调零命令会先确认机械臂静止，再把当前位置逐关节写为
零位并验证：

```bash
python -m arx_d_can.service_tools.zero_current_position --port /dev/ttyACM0
```

相同的安全调零流程也提供了编号示例：

```bash
python -m arx_d_can.examples.example_10_set_zero_current_position \
  --port /dev/ttyACM0
```

默认只调 6 个手臂关节；夹爪另加 `--include-gripper`。其他维护工具：

```bash
python -m arx_d_can.service_tools.change_damiao_id --port /dev/ttyACM0
python -m arx_d_can.service_tools.joint_load_probe \
  --port /dev/ttyACM0 --joint 4 --amplitude-deg 10 --csv /tmp/joint4.csv
```

## 配置

硬件 ID、反馈 ID、控制增益、夹爪映射和安全参数位于
`src/arx_d_can/config/arx_d_can_dm.yaml`。VR/ROS 上层已经负责工作空间和 URDF
关节限位；SDK 安全层负责通信故障、命令超时保持和退出失能。

## 开发验证

源码采用标准的 `src/arx_d_can` 包布局。安装开发依赖后可直接运行测试和构建：

```bash
python -m pip install ".[dev]"
python -m pytest --import-mode=importlib --rootdir=tests tests
python -m pip wheel --no-deps . --wheel-dir dist
```
