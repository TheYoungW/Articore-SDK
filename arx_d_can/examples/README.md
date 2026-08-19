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
  --opening 1000 \
  --speed 1000 \
  --force-level 5

python -m arx_d_can.examples.single_arm.example_11_gravity_compensation \
  --arm-model yunyi_v1_0_right \
  --transition-ms 1000

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
  --right-gripper 500 \
  --gripper-level 3

python -m arx_d_can.examples.dual_arm.example_15_gravity_compensation

python -m arx_d_can.examples.dual_arm.example_16_record_gravity_trajectory \
  --output trajectories/dual_gravity_demo.json \
  --seconds 30 \
  --hz 100

python -m arx_d_can.examples.dual_arm.example_17_replay_trajectory \
  --input trajectories/dual_gravity_demo.json \
  --mode pv \
  --interpolation quintic
```

PV 和 MIT 位置示例都调用 C++ Runtime 的普通位置接口，不在 Python 中插值。
MIT 目标速度和前馈力矩均为零，Kp/Kd 读取产品配置。两种模式的 `--velocity`
都是 `0～100` 的统一速度档位；Runtime 在 C++ 内换算物理速度并按控制周期限步，新值
覆盖旧的最终目标且不会排队。`0` 暂停，`100` 对应当前产品和模式的最大普通速度。
夹爪开合度必须显式填写，范围为 0～1000；0 表示闭合，1000 表示打开。
`--gripper-level` 范围为 1～5，默认 3，5 最强。速度和电机控制参数由 Runtime
产品配置固定管理。

示例 06/07 的限步、固定频率发送、通信检查和最终位置保持均由 C++ Runtime 处理；
调用本身非阻塞。SDK 不再公开第二套关节轨迹执行接口。

`set_joint_mit()` / `set_joint_pv()` 的重复调用频率不设用户侧上限：调用更快时 Runtime
在内部周期读取最新值，调用更慢时保持最近值，目标不会排队。

Yunyi 单臂和双臂都使用 Runtime 原生重力补偿，固定要求 MIT 模式和七轴产品绑定。
模型计算、渐入渐出、实际控制频率和逐周期力矩限制全部在 Runtime 内完成；Python 只
读取状态和录制反馈。双臂由同一个 Runtime 管理左右 14 轴。示例 16 同时录制左右臂实际反馈、夹爪和
真实采样时间戳；示例 17 从使能开始始终使用 raw PV，不在普通 PV 与 raw PV 之间
切换。`--mode pv|mit` 选择整次回放的 raw 控制路径；MIT 每帧使用插值位置、
`dq=0`、YAML Kp/Kd 和 `tau_ff=0`。它先用五次 S 曲线回到起点，再由 Runtime 自动
调度回放。`--interpolation`
支持 `none`（零阶保持）、`linear`（线性）和 `quintic`（五次 S 曲线），可用于同一
轨迹的对比实验。超过命令限位的录制反馈只在回放下发时裁剪，原始 JSON 不会被修改。
实机运行前请先激活已经安装 SDK 的 Python 环境，再直接执行上面的 `python -m ...`
命令。
