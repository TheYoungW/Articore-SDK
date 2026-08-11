# ARX-D-CAN 示例

示例按机械臂使用形态分组：

- `single_arm/`：完整的通用单臂示例，通过 `--arm-model` 选择机型；
- `dual_arm/`：与单臂相同的 01～12 双臂示例，当前默认使用 Yunyi V1.0。

单臂示例不会绑定某一种产品：

```bash
python -m arx_d_can.examples.single_arm.example_02_read_state \
  --arm-model yunyi_v1_0_right

python -m arx_d_can.examples.single_arm.example_04_send_position \
  --arm-model yunyi_v1_0_right \
  --positions "0,-20,-20,0,0,0,0"
```

双臂示例：

```bash
python -m arx_d_can.examples.dual_arm.example_02_read_state
```

夹爪固定模式、命令刷新、轨迹插值、防堵转、通信检查和退出失能均由 SDK 内部处理。

实机运行时先激活 Python 环境。若使用 `conda run`，必须添加
`--no-capture-output`，否则 Ctrl+C 可能被 Conda 外层截获，导致示例的退出失能
代码无法执行。
