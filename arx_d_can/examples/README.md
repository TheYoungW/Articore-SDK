# ARX-D-CAN Examples

普通用户按顺序运行这些示例。所有会使能电机的示例都会在退出、异常或
`Ctrl+C` 后尝试整臂失能。

```bash
sudo chmod 666 /dev/ttyACM0
cd Articore-SDK
python -m pip install -e .
```

## 01 扫描电机 ID

只检查通信，不使能电机：

```bash
python -m arx_d_can.examples.example_01_scan_ids --port /dev/ttyACM0
```

## 02 读取状态

```bash
python -m arx_d_can.examples.example_02_read_state --port /dev/ttyACM0
python -m arx_d_can.examples.example_02_read_state --port /dev/ttyACM0 --watch --hz 10
```

## 03 清除全部电机故障

先移除堵转障碍、释放负载并托住机械臂。该示例逐个清除六个关节的电机故障，
验证它们已进入失能状态，不会自动重新使能或运动：

```bash
python -m arx_d_can.examples.example_03_clear_faults --port /dev/ttyACM0
```

同时处理夹爪电机：

```bash
python -m arx_d_can.examples.example_03_clear_faults \
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
PV 可用 `--velocity-limits` 覆盖六个关节的最大速度，MIT 可用 `--velocities`
设置六个目标速度；这两个命令行参数的单位都是 deg/s，但控制语义不同。未提供时，
PV 使用 YAML 中各关节的 `vlim`（rad/s），MIT 的目标速度和力矩均默认为零。模式会
在电机使能前完成配置。

注意：MIT 的 `--velocities` 是控制公式中 `kd(v_des-v)` 的目标速度，不是最大速度
限制。示例 04 会直接发送位置目标；如果需要限制整个运动过程的速度，应使用示例 07
生成插值轨迹，而不是只修改 MIT 的目标速度。

```bash
python -m arx_d_can.examples.example_04_send_position \
  --positions "0,-20,-20,0,0,0" \
  --velocity-limits "120,120,120,90,90,90" \
  --port /dev/ttyACM0
```

使用 MIT 模式：

```bash
python -m arx_d_can.examples.example_04_send_position \
  --positions "0,-20,-20,0,0,0" \
  --mode mit \
  --velocities "0,0,0,0,0,0" \
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
python -m arx_d_can.examples.example_05_gripper_open_close --port /dev/ttyACM0
```

## 06 读取频率测试

```bash
python -m arx_d_can.examples.example_06_benchmark_read_rate \
  --port /dev/ttyACM0 --target-hz 500 --seconds 5
```

## 07 发送 500 Hz 平滑关节轨迹

默认使用五次最小加加速度时间缩放，以 500 Hz 从当前角度运动到目标。目标输入
单位为度，到位保持 2 秒后平滑回零：

```bash
python -m arx_d_can.examples.example_07_send_joint_trajectory \
  "0,-60,-60,0,0,0" \
  --port /dev/ttyACM0 \
  --duration 6 \
  --hz 500 \
  --return-zero
```

## 08 全部电机直接回到零位

直接向六个机械臂关节发送 `0°`，夹爪默认也发送电机 `0°`。该示例不做插值，
只发送位置命令，不会修改电机零点标定。使能后会逐轴验证 `ENABLED` 反馈，运行中
每秒打印实际关节角并监测电机故障。默认持续刷新零目标，按 `Ctrl+C` 后全部失能。
夹爪零位对应闭合，运行前必须确保夹爪内没有物体：

```bash
python -m arx_d_can.examples.example_08_return_zero --port /dev/ttyACM0
```

可用 `--velocity-limit` 设置六轴统一的 PV 最大速度（单位 `deg/s`），例如：

```bash
python -m arx_d_can.examples.example_08_return_zero \
  --port /dev/ttyACM0 \
  --velocity-limit 15
```

目标会直接交给电机位置控制器；运行前必须确认当前位置到零位之间没有碰撞风险。
需要平滑运动到零位时，使用示例 07 并把目标位置设为六个 `0`。

只移动六个机械臂关节、不连接夹爪：

```bash
python -m arx_d_can.examples.example_08_return_zero \
  --port /dev/ttyACM0 \
  --arm-only
```

修改电机 ID 和负载轨迹测试属于维护操作，位于 `arx_d_can.service_tools` 子包。
调零可通过下面的示例 10 运行，但同样属于维护操作，不要作为普通用户首次操作运行。
清除故障不能修复仍然存在的堵转、过热、过流或欠压原因；故障源没有消失时，电机在
后续使能后仍会再次进入故障。

## 09 诊断失能、故障和控制模式

只读取所有电机的反馈状态、MOS/绕组温度以及 `CTRL_MODE` 寄存器。不会使能、
失能、切换模式、清除故障或发送运动命令：

```bash
python -m arx_d_can.examples.example_09_diagnose_status \
  --port /dev/ttyACM3
```

模式值：`1=MIT`、`2=POS_VEL`、`3=VEL`、`4=FORCE_POS`。状态值 `0` 是正常
失能，`1` 是正常使能；其他值会输出对应故障名称。存在故障、温度异常或反馈
不完整时，不要直接清故障或使能，先检查硬件。

## 10 将当前位置设为电机零位

该示例不会驱动机械臂运动，而是把当前静止位置写入电机作为新的零位。运行前确认
机械臂已经放在机械零位并保持静止；命令会检查反馈和静止状态，然后直接写入六个
机械臂关节。每个电机写入后必须连续收到 3 帧状态正常、位置接近零且速度接近零的
新反馈，才判定调零成功：

```bash
python -m arx_d_can.examples.example_10_set_zero_current_position \
  --port /dev/ttyACM0
```

默认不调夹爪；如需同时将夹爪当前位置设为零，增加 `--include-gripper`。调零会修改
电机持久零点，执行前必须托稳机械臂并确认各关节处于正确的机械零位。

## 11 录制和回放轨迹

录制时不会使能电机，可手动拖动机械臂。默认以 100 Hz 录制 10 秒，频率最高
500 Hz，六个关节和夹爪位置保存在 JSON 文件中：

```bash
python -m arx_d_can.examples.example_11_record_and_replay_trajectory \
  record trajectory.json \
  --seconds 10 \
  --hz 100 \
  --port /dev/ttyACM0
```

回放不需要再指定频率，会自动按照文件中记录的频率执行：

```bash
python -m arx_d_can.examples.example_11_record_and_replay_trajectory \
  replay trajectory.json \
  --port /dev/ttyACM0
```

回放会直接发送记录的第一个机械臂和夹爪位置。执行前应托稳机械臂，确认当前位置与
夹爪开合程度接近轨迹起点，并确保整条轨迹没有碰撞和夹伤风险。
