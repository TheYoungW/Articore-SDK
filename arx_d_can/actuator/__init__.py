"""actuator 模块 — JointGroup 架构（分组控制，同步发送）。

默认机型和内置机型列表由 config/models.yaml 定义；每个机型使用独立硬件 YAML。

示例::

    arx_d_can = ArxDCan()   # 自动读取 models.yaml 中的默认机型
    arx_d_can.connect()
    arx_d_can.arm.enable()
    arx_d_can.gripper.enable()
    arx_d_can.arm.mode_pos_vel()       # arm 组切换模式
    arx_d_can.gripper.mode_mit()       # gripper 组切换模式

    def loop(r, dt):
        r.arm.send_pos_vel(joint_pos)     # arm 组发送
        r.gripper.send_mit(gripper_pos)   # gripper 组发送

    arx_d_can.start_control_loop(loop)
    arx_d_can.stop_control_loop()
    arx_d_can.disconnect()
"""

from .arx_d_can import ArxDCan, JointGroup, JointCfg, available_models, load_cfg

__all__ = [
    "ArxDCan",
    "JointGroup",
    "JointCfg",
    "available_models",
    "load_cfg",
]
