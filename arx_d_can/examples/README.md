# ARX-D-CAN 示例

示例只展示用户需要调用的高层接口。轨迹插值、命令刷新、夹爪防堵转、通信检查和
安全失能均由 SDK 内部完成。

所有命令都可以使用 `--arm-model yunyi_v1_0_left` 或
`--arm-model yunyi_v1_0_right` 选择机型，并用 `--port` 覆盖配置中的通信端口。

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

`--mode` 只支持 `pv` 和 `mit`。示例会持续刷新目标，按 `Ctrl+C` 后自动失能。

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

这是维护操作，会修改电机持久零点。执行前必须确认机械臂静止并位于正确机械零位。

## 11 录制和回放轨迹

```bash
python -m arx_d_can.examples.example_11_record_and_replay_trajectory \
  record trajectory.json --seconds 10 --port /dev/ttyACM0

python -m arx_d_can.examples.example_11_record_and_replay_trajectory \
  replay trajectory.json --port /dev/ttyACM0
```

## 12 重力补偿

```bash
python -m arx_d_can.examples.example_12_gravity_compensation \
  --arm-model yunyi_v1_0_right \
  --port /dev/ttyACM0 \
  --seconds 10
```

重力补偿会使机械臂可被手动拖动，运行前必须托稳机械臂并远离关节限位。

## 13 关节行程测试

这是面向维护人员的薄入口，复杂校验位于 `arx_d_can.service_tools`，不属于普通
Yunyi 用户的首次使用流程。

```bash
python -m arx_d_can.examples.example_13_test_joint_range --help
```
