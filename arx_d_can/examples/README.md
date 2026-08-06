# ARX-D-CAN Examples

普通用户按顺序运行这些示例。所有会使能电机的示例都会在退出、异常或
`Ctrl+C` 后尝试整臂失能。

```bash
sudo chmod 666 /dev/ttyACM0
cd Articore-SDK
python -m pip install -e .
```

所有示例默认使用 `arx_d_can/config/models.yaml` 中的 `default_model`。选择其他
内置机型时增加 `--arm-model <名称>`；临时使用外部硬件配置时增加
`--config-path /path/to/arm.yaml`。两个参数不能同时使用。`--port` 与 `--channel`
等价，配合 `--transport dm-serial|socketcan|socketcanfd` 可覆盖机型 YAML 中的
连接参数；`--baud` 只对 dm-serial 有效。SocketCAN 的完整说明见
[SocketCAN 使用说明](../../docs/socketcan.md)。示例会按所选机型的实际关节数量校验输入。

## 01 扫描电机 ID

只检查通信，不使能电机：

```bash
python -m arx_d_can.examples.example_01_scan_ids --port /dev/ttyACM0
```

板载经典 CAN（例如 RK3588 + MCP2515）的扫描命令：

```bash
python -m arx_d_can.examples.example_01_scan_ids \
  --transport socketcan \
  --channel can0 \
  --feedback-base 0x200
```

预期反馈 ID 按 `feedback_base + (motor_id & 0x0F)` 计算。电机 ID `6`、反馈 ID
`0x206` 必须使用 `--feedback-base 0x200`；现有 `0x11`～`0x1F` 映射使用默认
`0x10`。扫描结果会校验实际仲裁 ID，并过滤 255°C 和编码极值等无效状态，防止把
其他电机的反馈帧误报为当前候选 ID。

## 02 读取状态

默认以 10 Hz 持续打印，按 `Ctrl+C` 停止。使用 `--once` 只读取一次：

```bash
python -m arx_d_can.examples.example_02_read_state --port /dev/ttyACM0
python -m arx_d_can.examples.example_02_read_state --port /dev/ttyACM0 --once
```

## 03 清除全部电机故障

先移除堵转障碍、释放负载并托住机械臂。该示例逐个清除所选机型各关节的电机故障，
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
关节配置的 `kp/kd`，还可通过 `--torques` 设置各关节的前馈力矩（N·m）。
PV 可用 `--velocity-limits` 覆盖各关节的最大速度，MIT 可用 `--velocities`
设置各关节目标速度；这两个命令行参数的单位都是 deg/s，但控制语义不同。未提供时，
PV 使用 YAML 中各关节的 `vlim`（rad/s），MIT 的目标速度和力矩均默认为零。模式会
在电机使能前完成配置。

### MIT 零增益和纯力矩发送

`--torques` 是前馈力矩，示例 04 仍同时使用 YAML 的 Kp/Kd，并非纯力矩控制。
示例 04 的命令行目前没有 Kp/Kd 覆盖参数；需要纯力矩控制时必须使用 Python API，
并且显式传入 `mit_kp=0`、`mit_kd=0`：

```python
arm.send_joint_positions(
    positions,
    torques=torques,
    mit_kp=0,
    mit_kd=0,
    mode="mit",
)
```

标量 `0` 会应用到全部机械臂关节，而且不会被当成“未设置”。不要通过省略参数来
表示零增益：省略 `mit_kp` 或 `mit_kd` 会使用 YAML 中对应的默认增益。也可以传入
与机械臂关节数相同的数组，以便逐关节设置增益，例如
`mit_kp=[0.0] * len(arm.joint_names)`。

当 Kp 为零时，仍需传入 `positions` 以组成完整 MIT 数据帧，但位置误差不会产生
控制力矩；当 Kd 也为零时，电机只执行 `torques` 指定的前馈力矩。纯力矩模式必须
持续刷新指令，并自行实施力矩限幅、速度监测、关节限位和通信异常处理。命令超时后，
SDK 看门狗会在最后一次成功发送的位置恢复 YAML 默认 MIT 增益进行安全保持，而不是
继续保持零增益。

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

直接向所选机型的全部机械臂关节发送 `0°`，夹爪默认也发送电机 `0°`。该示例不做插值，
只发送位置命令，不会修改电机零点标定。使能后会逐轴验证 `ENABLED` 反馈，运行中
每秒打印实际关节角并监测电机故障。默认持续刷新零目标，按 `Ctrl+C` 后全部失能。
夹爪零位对应闭合，运行前必须确保夹爪内没有物体：

```bash
python -m arx_d_can.examples.example_08_return_zero --port /dev/ttyACM0
```

可用 `--velocity-limit` 设置全部关节统一的 PV 最大速度（单位 `deg/s`），例如：

```bash
python -m arx_d_can.examples.example_08_return_zero \
  --port /dev/ttyACM0 \
  --velocity-limit 15
```

目标会直接交给电机位置控制器；运行前必须确认当前位置到零位之间没有碰撞风险。
需要平滑运动到零位时，使用示例 07 并为全部关节传入 `0`。

只移动机械臂关节、不连接夹爪：

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
机械臂已经放在机械零位并保持静止；命令会检查反馈和静止状态，然后直接写入所选
机型的全部机械臂关节。每个电机写入后必须连续收到 3 帧状态正常、位置接近零且速度接近零的
新反馈，才判定调零成功：

```bash
python -m arx_d_can.examples.example_10_set_zero_current_position \
  --port /dev/ttyACM0
```

使用 `--joint` 可只写入指定关节，避免改变其他关节已经校准好的持久零点：

```bash
python -m arx_d_can.examples.example_10_set_zero_current_position \
  --arm-model yunyi_v1_0_left \
  --port /dev/ttyACM4 \
  --joint l-joint4
```

默认不调夹爪；如需同时将夹爪当前位置设为零，增加 `--include-gripper`。调零会修改
电机持久零点，执行前必须托稳机械臂并确认各关节处于正确的机械零位。

## 11 录制和回放轨迹

录制时不会使能电机，可手动拖动机械臂。默认按墙钟时间录制 10 秒，并以 100 Hz
为目标采样频率，目标频率最高 500 Hz。JSON 会保存每个样本的真实相对时间戳以及
全部机械臂关节和夹爪位置；处理超期时会跳过过期周期，不会延长录制时间补足样本：

```bash
python -m arx_d_can.examples.example_11_record_and_replay_trajectory \
  record trajectory.json \
  --seconds 10 \
  --hz 100 \
  --port /dev/ttyACM0
```

如果失能状态下机械臂不易拖动，可增加 `--enable`。录制进程会使用 MIT 模式使能，
并持续发送 `Kp=0、Kd=0、torque=0` 的零刚度命令。该模式不计算或发送重力补偿
力矩；录制结束或异常退出时自动失能：

```bash
python -m arx_d_can.examples.example_11_record_and_replay_trajectory \
  record trajectory.json \
  --seconds 10 \
  --hz 100 \
  --arm-model yunyi_v1_0_left \
  --port /dev/ttyACM4 \
  --enable
```

MIT 数据帧包含当前位置，但由于 `Kp=0`，不会产生位置保持力。使能前必须托稳
机械臂，并与关节限位保持距离。

需要在 Pinocchio 重力补偿状态下示教录制时，使用
`--gravity-compensation`。录制循环会以指定频率实时更新重力力矩并保存关节和夹爪
位置，结束时自动失能：

```bash
python -m arx_d_can.examples.example_11_record_and_replay_trajectory \
  record yunyi_left_trajectory.json \
  --seconds 10 \
  --hz 200 \
  --arm-model yunyi_v1_0_left \
  --port /dev/ttyACM4 \
  --gravity-compensation
```

回放不需要再指定频率，新格式轨迹会严格按照文件中的逐点时间戳执行。旧格式文件
没有时间戳，只能继续按照其中的名义频率执行：

```bash
python -m arx_d_can.examples.example_11_record_and_replay_trajectory \
  replay trajectory.json \
  --port /dev/ttyACM0
```

回放会直接发送记录的第一个机械臂和夹爪位置。执行前应托稳机械臂，确认当前位置与
夹爪开合程度接近轨迹起点，并确保整条轨迹没有碰撞和夹伤风险。

## 12 重力补偿模式

示例 12 使用 MIT 模式实时计算并发送 URDF 重力力矩。活动阶段默认显式发送
`Kp=0、Kd=0`，机械臂不会保持位置，必须先托住机械臂并远离关节限位。程序会从
使能后默认直接切换到重力补偿，退出时恢复 YAML MIT 增益，然后失能全部机械臂
关节。如需先保持当前位置或渐变接管，可显式设置 `--settle-seconds` 或
`--transition-seconds`：

```bash
python -m arx_d_can.examples.example_12_gravity_compensation \
  --arm-model yunyi_v1_0_right \
  --port /dev/ttyACM0 \
  --seconds 10
```

`--seconds 0` 表示持续运行到按下 `Ctrl+C`。如果需要保留少量速度阻尼，可增加
`--damping 0.2`；此时仍保持 `Kp=0`，但 Kd 不再为零。`--gravity-scale` 设置整体
重力力矩倍率，`--joint-scales` 设置逐关节倍率。例如 7 轴机械臂可传入：

```bash
python -m arx_d_can.examples.example_12_gravity_compensation \
  --arm-model yunyi_v1_0_right \
  --port /dev/ttyACM0 \
  --joint-scales "1,1.55,1.55,1,1,1,1"
```

该模式不会针对关节速度、URDF 关节范围或 Pinocchio 重力力矩设置 SDK 软件阈值；
通信、电机故障和无效反馈仍会中止运行。运行该示例需要安装 `pin>=3.0`；如果当前
shell 加载的 ROS `PYTHONPATH` 覆盖了 conda 环境中的 Pinocchio，可用
`env -u PYTHONPATH` 放在命令前。

## 13 单关节 URDF 范围测试

示例 13 默认只连接并使能 Corina 右腿关节 1–4。程序逐个关节执行“当前位置、下侧
测试点、上侧测试点、返回当前位置”，当前被测关节运动时，其余三个关节保持初始
位置。默认测试到零位朝每侧 URDF 极限方向的 95%，每段轨迹 6 秒、200 Hz：

```bash
python -m arx_d_can.examples.example_13_test_joint_range \
  --arm-model corina_v2 \
  --port /dev/ttyACM5
```

程序使能前会打印当前角度、URDF 上下限和实际测试目标；当前反馈超出 URDF、关节
缺少上下限或目标跟踪误差超过 1 度时会停止并失能。可用 `--range-percent 80` 修改
行程比例，或用 `--joints` 指定其他关节。关节5/6应通过常规多关节轨迹接口按
Pitch/Roll 坐标控制，不属于这个逐个关节维修测试示例的默认测试对象。
