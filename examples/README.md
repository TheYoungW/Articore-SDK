# ARX-D-CAN Examples

普通用户按顺序运行这些示例。所有会使能电机的示例都会在退出、异常或
`Ctrl+C` 后尝试整臂失能。

```bash
sudo chmod 666 /dev/ttyACM0
cd arx_d_can
python -m pip install -e .
```

## 01 扫描电机 ID

只检查通信，不使能电机：

```bash
python examples/example_01_scan_ids.py --port /dev/ttyACM0
```

## 02 读取状态

```bash
python examples/example_02_read_state.py --port /dev/ttyACM0
python examples/example_02_read_state.py --port /dev/ttyACM0 --watch --hz 10
```

## 03 清除全部电机故障

先移除堵转障碍、释放负载并托住机械臂。该示例逐个清除六个关节的电机故障，
验证它们已进入失能状态，不会自动重新使能或运动：

```bash
python examples/example_03_clear_faults.py --port /dev/ttyACM0
```

同时处理夹爪电机：

```bash
python examples/example_03_clear_faults.py \
  --port /dev/ttyACM0 \
  --include-gripper
```

## 04 发送关节位置

这个示例只直接下发关节位置，不做插值或回零。关节位置输入单位统一为度。
目标发出后默认以 100 Hz 一直刷新，`--hold-seconds 0` 表示一直保持（默认）；
只有明确传入正数时，才会在指定时间后退出并整臂失能。需要平滑轨迹时使用
示例 07，直接回零时使用示例 08。

控制模式通过 `--mode` 选择：`pv` 是默认值，对应电机的 POS_VEL 模式；`mit`
对应 MIT 模式。PV 使用配置文件中的位置环、速度环和速度限制参数，MIT 使用各
关节配置的 `kp/kd`，还可通过 `--torques` 设置六个关节各自的前馈力矩（N·m）。
未提供力矩时 MIT 默认发送六个零；PV 模式不接受 `--torques`。模式会在电机使能
前完成配置。

```bash
python examples/example_04_send_position.py \
  --positions "0,-20,-20,0,0,0" \
  --port /dev/ttyACM0
```

使用 MIT 模式：

```bash
python examples/example_04_send_position.py \
  --positions "0,-20,-20,0,0,0" \
  --mode mit \
  --torques "0,0,0,0,0,0" \
  --port /dev/ttyACM0
```

该目标会直接交给电机位置控制器；运行前必须确认当前位置到目标位置之间没有
碰撞风险，并避免一次发送跨度过大的目标。

默认需要用 `Ctrl+C` 停止。停止会失能全部电机，因此按下前必须托住机械臂或使用
可靠的机械防坠装置。如果确实要保持 10 秒后自动失能，可以显式传入
`--hold-seconds 10`。

## 05 夹爪开合

```bash
python examples/example_05_gripper_open_close.py --port /dev/ttyACM0
```

## 06 读取频率测试

```bash
python examples/example_06_benchmark_read_rate.py \
  --port /dev/ttyACM0 --target-hz 500 --seconds 5
```

## 07 发送 500 Hz 平滑关节轨迹

默认使用五次最小加加速度时间缩放，以 500 Hz 从当前角度运动到目标。目标输入
单位为度，到位保持 2 秒后平滑回零：

```bash
python examples/example_07_send_joint_trajectory.py \
  "0,-60,-60,0,0,0" \
  --port /dev/ttyACM0 \
  --duration 6 \
  --hz 500 \
  --return-zero
```

## 08 全部电机直接回到零位

直接向六个机械臂关节发送 `0°`，夹爪默认也发送电机 `0°`。该示例不做插值，
只发送位置命令，不会修改电机零点标定。默认持续刷新零目标，按 `Ctrl+C` 后
全部失能。夹爪零位对应闭合，运行前必须确保夹爪内没有物体：

```bash
python examples/example_08_return_zero.py --port /dev/ttyACM0
```

目标会直接交给电机位置控制器；运行前必须确认当前位置到零位之间没有碰撞风险。
需要平滑运动到零位时，使用示例 07 并把目标位置设为六个 `0`。

只移动六个机械臂关节、不连接夹爪：

```bash
python examples/example_08_return_zero.py \
  --port /dev/ttyACM0 \
  --arm-only
```

调零、修改电机 ID 和负载轨迹测试属于维护操作，已移到 `service_tools/`，不要
作为普通用户首次操作运行。清除故障不能修复仍然存在的堵转、过热、过流或欠压
原因；故障源没有消失时，电机在后续使能后仍会再次进入故障。

## 09 诊断失能、故障和控制模式

只读取所有电机的反馈状态、MOS/绕组温度以及 `CTRL_MODE` 寄存器。不会使能、
失能、切换模式、清除故障或发送运动命令：

```bash
python examples/example_09_diagnose_status.py \
  --port /dev/ttyACM3
```

模式值：`1=MIT`、`2=POS_VEL`、`3=VEL`、`4=FORCE_POS`。状态值 `0` 是正常
失能，`1` 是正常使能；其他值会输出对应故障名称。存在故障、温度异常或反馈
不完整时，不要直接清故障或使能，先检查硬件。
