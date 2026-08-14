# ARX-D-CAN 示例

示例按机械臂使用形态分组：

- `single_arm/`：完整的通用单臂示例，通过 `--arm-model` 选择机型；
- `dual_arm/`：双臂示例，当前默认使用 Yunyi V1.0；示例 02 切换模式，示例 03
  演示使能/失能，示例 06 为 PV，示例 07 为 MIT。

单臂示例不会绑定某一种产品：

```bash
python -m arx_d_can.examples.single_arm.example_02_read_state \
  --arm-model yunyi_v1_0_right

python -m arx_d_can.examples.single_arm.example_04_send_position \
  --arm-model yunyi_v1_0_right \
  --positions "0,-20,-20,0,0,0,0"

python -m arx_d_can.examples.single_arm.example_05_set_gripper_opening \
  --arm-model yunyi_v1_0_right \
  --opening 1000

python -m arx_d_can.examples.single_arm.example_13_record_gravity_trajectory \
  --arm-model yunyi_v1_0_left \
  --output trajectories/left_gravity_demo.json \
  --seconds 30 \
  --hz 100
```

双臂示例：

```bash
python -m arx_d_can.examples.dual_arm.example_02_switch_control_mode --mode mit

python -m arx_d_can.examples.dual_arm.example_03_enable_disable

python -m arx_d_can.examples.dual_arm.example_04_read_state

python -m arx_d_can.examples.dual_arm.example_04_read_state \
  --mode continuous  # 100 Hz 采集、默认 10 Hz 显示，按 Ctrl+C 停止

python -m arx_d_can.examples.dual_arm.example_06_send_position_pv \
  --left "0,0,0,90,0,0,0" \
  --right "0,0,0,90,0,0,0" \
  --velocity 60

python -m arx_d_can.examples.dual_arm.example_07_send_position_mit \
  --left "0,0,0,90,0,0,0" \
  --right "0,0,0,90,0,0,0" \
  --velocity 60

python -m arx_d_can.examples.dual_arm.example_08_set_gripper_openings \
  --left-gripper 1000 \
  --right-gripper 500

python -m arx_d_can.examples.single_arm.example_12_gravity_compensation

python -m arx_d_can.examples.dual_arm.example_15_gravity_compensation
```

PV 和 MIT 位置示例都调用 C++ Runtime 的普通位置接口，不在 Python 中插值。
MIT 目标速度和前馈力矩均为零，Kp/Kd 读取机型 YAML。两种模式的 `--velocity`
都是必须填写的统一实际速度，命令行单位为度/秒；SDK 转为 rad/s 后一次提交完整
双臂目标。Runtime 以原生 500 Hz 限步，新值覆盖旧的最终目标且不会排队。普通 MIT
的产品级速度范围为 `(0, 200]` 度/秒；URDF/YAML `vlim` 仍作为更底层的绝对上限。
夹爪开合度必须显式填写，范围为 0～1000；0 表示闭合，1000 表示打开。

示例 06/07 的限步、固定频率发送、通信检查和最终位置保持均由 C++ Runtime 处理；
调用本身非阻塞。SDK 不再公开第二套关节轨迹执行接口。

重力补偿示例内部固定使用 MIT 模式和机型参数。用户无需填写 Kp/Kd、补偿比例或
发送频率；Python 计算 URDF 重力矩，底层 Runtime 负责 500 Hz 发送和安全状态机。

示例 13 在重力补偿下录制实际关节反馈和夹爪位置，并保存真实采样时间戳。输出
文件夹不存在时会自动创建。实机运行所有示例前，请先激活已经安装 SDK 的 Python
环境，再直接执行上面的 `python -m ...` 命令。
