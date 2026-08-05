# SocketCAN 使用说明

Articore-SDK 支持直接使用 Linux SocketCAN。适用场景包括 RK3588 上的 MCP2515、
板载 CAN 控制器，以及由其他驱动暴露出的 `can0`/`can1`。MCP2515 是经典 CAN
控制器，应使用 `transport: socketcan`，不要使用 `socketcanfd`。

## 1. 依赖和版本

项目固定依赖 `motor-drive-layer==0.5.4`。安装或更新项目后确认版本：

```bash
python -m pip install -e .
python -m pip show motor-drive-layer
```

若使用 conda 环境，先激活项目环境，再运行上述命令。不要混用系统 Python 与 conda
Python；可用 `which python` 和 `python -m pip --version` 确认两者属于同一环境。

## 2. 配置 Linux CAN 接口

以下命令把 `can0` 设置为经典 CAN 1 Mbps：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0
```

正常通信时应看到接口为 `UP`，CAN 状态为 `ERROR-ACTIVE`。`ERROR-PASSIVE`、
`BUS-OFF` 或持续增长的 error counter 通常是终端电阻、波特率、CAN_H/CAN_L、
共地或供电问题，不是 Python 串口参数问题。

SocketCAN 模式下 YAML 的 `baud` 和命令行 `--baud` 不配置 CAN 总线速率；总线速率
只由上面的 `ip link` 命令决定。

可先用 `can-utils` 验证原始总线：

```bash
candump can0
ip -details -statistics link show can0
```

## 3. 扫描电机 ID

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

`--port can0` 与 `--channel can0` 等价。扫描过程中，每个候选电机的预期反馈 ID
按下面的公式计算：

```text
expected_feedback_id = feedback_base + (motor_id & 0x0F)
```

因此，电机 ID `6`、反馈 ID `0x206` 使用 `--feedback-base 0x200`；电机 ID
`1`、反馈 ID `0x11` 使用默认值 `0x10`。若电机的 MST_ID 不符合这一组连续映射，
扫描器无法仅根据 ESC_ID 推导它，必须先获知实际反馈 ID，再把它写入机型 YAML 或用
底层单电机命令指定 `--motor-id` 和 `--feedback-id`。

SDK 不会直接相信底层 CLI 输出的 `[hit]`。结果还必须同时满足：

- 实际 `arbitration_id` 与本次候选的预期反馈 ID 完全一致；
- 状态码位于 Damiao 的有效范围；
- 位置、速度、力矩和温度均为有限数；
- 温度不是 `255°C` 等无效哨兵值；
- 位置、速度和力矩不会同时处于所选电机型号的编码极值。

这会过滤“候选 ID 15 收到了其他电机的帧”一类假阳性。若已知电机没有被扫描到，
先检查 `--model`、`--feedback-base` 和 Linux CAN 错误计数，不要盲目扩大 ID 范围。

## 4. YAML 配置

自定义机型配置中明确写出后端和通道：

```yaml
name: RK3588 Arm
transport: socketcan
channel: can0
baud: 1000000  # 仅为兼容字段；SocketCAN 模式下不用于设置总线比特率
rate: 500
```

内置机型目前明确配置为 `transport: dm-serial`。使用同一机型但临时改走板载 CAN
时不必编辑内置 YAML，可直接通过命令行覆盖：

```bash
python -m arx_d_can.examples.example_02_read_state \
  --arm-model yunyi_v1_0_right \
  --transport socketcan \
  --channel can0 \
  --once
```

## 5. Python API

读状态和运动控制使用同一个显式连接参数：

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

上例的运动部分会使能整臂并持续保持当前位置，只能在机械臂已处于安全环境、有人托稳
且具备急停时运行。只验证通信时，到 `print(arm.read_state())` 为止，不要调用
`configure()` 或 `enable()`。

低层 API 也支持相同配置：

```python
from arx_d_can import ArxDCan

arm = ArxDCan(model="yunyi_v1_0_right", transport="socketcan", channel="can0")
arm.connect()
```

## 6. 常见问题

- `No such device`：Linux 没有名为 `can0` 的接口，先检查 `ip link` 和设备树驱动。
- 扫描结果为 `none`：优先检查接口是否 `UP`、1 Mbps 是否一致、反馈 ID 基址和电机
  型号是否正确。
- 扫描到错误 ID：新版 SDK 会校验实际仲裁 ID；若仍出现，保存完整扫描输出和
  `candump can0`，确认总线上是否有重复反馈 ID。
- `BUS-OFF`：先修复物理层和波特率，再重新拉起接口；应用层重试不能修复总线错误。
- `/dev/ttyACM0`：这是 dm-serial 设备，不是 SocketCAN。应使用
  `--transport dm-serial --channel /dev/ttyACM0`。
- `can0`：这是网络接口，不是文件路径。应使用
  `--transport socketcan --channel can0`，不需要也不应改 `/dev/ttyACM0` 权限。
