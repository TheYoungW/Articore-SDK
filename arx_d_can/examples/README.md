# ARX-D-CAN 示例

示例只展示用户需要调用的高层接口。轨迹插值、命令刷新、夹爪防堵转、通信检查和
安全失能均由 SDK 内部完成。调用 `enable()` 时 SDK 会自动配置所选控制模式，普通
用户不需要额外调用 `configure()`。

所有示例默认使用 `yunyi_v1_0_right`，直接传入 `--port` 即可运行。左臂使用
`--arm-model yunyi_v1_0_left`，普通用户不需要指定自定义配置文件。

## 01 扫描电机 ID

只扫描通信，不使能电机：

```bash
python -m arx_d_can.examples.example_01_scan_ids --port /dev/ttyACM0
```

## 02 读取状态

```bash
python -m arx_d_can.examples.example_02_read_state --port /dev/ttyACM0
```

对应的 Python 调用只有一行：

```python
state = arm.read_state()
```

## 03 清除电机故障

```bash
python -m arx_d_can.examples.example_03_clear_faults --port /dev/ttyACM0
```

默认同时处理机械臂和夹爪。清除故障前必须先排除堵转、过热、欠压等原因。

## 04 发送关节位置

```bash
python -m arx_d_can.examples.example_04_send_position \
  --positions "0,-20,-20,0,0,0,0" \
  --mode pv \
  --port /dev/ttyACM0
```

`--mode` 只支持 `pv` 和 `mit`。示例会停留在最后目标位置，并由 SDK 在内部持续刷新
最后一帧；按 `Ctrl+C` 后自动失能。

```python
arm.hold_joint_positions(target)
```

## 05 夹爪开合

```bash
python -m arx_d_can.examples.example_05_gripper_open_close --port /dev/ttyACM0
```

夹爪使用 `0～1000` 的统一开合度，固定采用 MIT 模式和机型默认增益。防堵转逻辑
由 SDK 内部处理。

```python
arm.move_gripper(1000)  # 张开
arm.move_gripper(0)     # 闭合
```

## 06 读取频率测试

```bash
python -m arx_d_can.examples.example_06_benchmark_read_rate \
  --port /dev/ttyACM0 --hz 500 --seconds 5
```

## 07 平滑关节运动

```bash
python -m arx_d_can.examples.example_07_send_joint_trajectory \
  "0,-60,-60,0,0,0,0" \
  --seconds 6 \
  --port /dev/ttyACM0
```

```python
state = arm.move_joint_positions(target, seconds=6)
```

轨迹插值和发送节拍由 SDK 完成。增加 `--return-zero` 可在到位后平滑返回零位。

## 08 返回零位

```bash
python -m arx_d_can.examples.example_08_return_zero \
  --port /dev/ttyACM0 --seconds 6
```

该示例不会修改电机零点标定，只会让机械臂和夹爪平滑运动到零位置。

## 09 读取电机诊断

```bash
python -m arx_d_can.examples.example_09_diagnose_status --port /dev/ttyACM0
```

```python
diagnostics = arm.read_motor_diagnostics()
```

诊断是只读操作，不会使能、失能、切换模式或清除故障。

## 10 设置电机零点

```bash
python -m arx_d_can.examples.example_10_set_zero_current_position \
  --port /dev/ttyACM0
```

这是维护操作，会修改机械臂电机的持久零点。运行前先在电机失能状态下将机械臂手动
摆到正确机械零位；示例只连接、调零和断开，不会使能机械臂，也不会修改夹爪零点。

## 11 录制和回放轨迹

```bash
python -m arx_d_can.examples.example_11_record_and_replay_trajectory \
  record trajectory.json --seconds 10 --port /dev/ttyACM0

python -m arx_d_can.examples.example_11_record_and_replay_trajectory \
  replay trajectory.json --port /dev/ttyACM0
```

录制时电机保持失能，用户可以手动拖动机械臂和夹爪；回放时才会自动使能，并在结束
或异常退出后自动失能。基础示例只保留录制和回放，高级的重力补偿录制由维护工具提供。

## 12 重力补偿

```bash
python -m arx_d_can.examples.example_12_gravity_compensation \
  --arm-model yunyi_v1_0_right \
  --port /dev/ttyACM0 \
  --seconds 10
```

重力补偿会使机械臂可被手动拖动，运行前必须托稳机械臂并远离关节限位。
