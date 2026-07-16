"""actuator 模块 — JointGroup 架构（分组控制，同步发送）。

所有参数均在 config/arx_d_can.yaml 中定义，hardware_yaml 字段指定硬件配置文件。

示例::

    arx_d_can = ArxDCan()   # 自动从 arx_d_can.yaml 读取 hardware_yaml
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

from .arx_d_can import ArxDCan, JointGroup, JointCfg, load_cfg

__all__ = [
    "ArxDCan",
    "JointGroup",
    "JointCfg",
    "load_cfg",
]
