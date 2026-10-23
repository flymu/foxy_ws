#!/usr/bin/env python3
"""
go2_odom_bridge.py — Go2 四足解算位姿桥（Orin 上跑）
SDK2 CycloneDDS(eth1) 订阅固件 rt/lf/sportmodestate → rclpy FastDDS 发布 /go2/odom
（go2_bridge 同款双栈架构：固件只与 CycloneDDS 互通，工作站只与 FastDDS 互通，
 一个进程两条栈。20Hz 实测由 CycloneDDS-RMW 版验证，本版把数据直接发到 FastDDS 域。）

帧名独立于 LIO 主栈（go2_odom/go2_base，不碰 map/camera_init/body）。
四元数顺序 Unitree [w,x,y,z] → ROS [x,y,z,w]。
"""
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize

# SDK2 精简版 idl 缺 SportModeState/TimeSpec，按 rosidl 生成的 wire 定义补。
# XTypes 会比较字段名：必须用 rosidl wire 的带下划线字段（stamp_/position_...），
# SDK2 自带的 IMUState_（无下划线）不能复用，一并手写。
@dataclass
@annotate.final
@annotate.autoid("sequential")
class TimeSpec_(idl.IdlStruct, typename="unitree_go.msg.dds_.TimeSpec_"):
    sec_: types.int32          # rosidl: long sec_
    nanosec_: types.uint32

@dataclass
@annotate.final
@annotate.autoid("sequential")
class IMUState_(idl.IdlStruct, typename="unitree_go.msg.dds_.IMUState_"):
    quaternion_: types.array[types.float32, 4]
    gyroscope_: types.array[types.float32, 3]
    accelerometer_: types.array[types.float32, 3]
    rpy_: types.array[types.float32, 3]
    temperature_: types.uint8  # rosidl: octet temperature_

@dataclass
@annotate.final
@annotate.autoid("sequential")
class SportModeState_(idl.IdlStruct, typename="unitree_go.msg.dds_.SportModeState_"):
    stamp_: TimeSpec_
    error_code_: types.uint32
    imu_state_: IMUState_
    mode_: types.uint8
    progress_: types.float32
    gait_type_: types.uint8
    foot_raise_height_: types.float32
    position_: types.array[types.float32, 3]
    body_height_: types.float32
    velocity_: types.array[types.float32, 3]
    yaw_speed_: types.float32
    range_obstacle_: types.array[types.float32, 4]
    foot_force_: types.array[types.int16, 4]
    foot_position_body_: types.array[types.float32, 12]
    foot_speed_body_: types.array[types.float32, 12]


class Go2OdomBridge(Node):
    def __init__(self):
        super().__init__('go2_odom_bridge')
        self.pub = self.create_publisher(Odometry, '/go2/odom', 10)
        self.tfb = TransformBroadcaster(self)
        self.n = 0
        self.get_logger().info('go2_odom_bridge ready')

    def on_state(self, m):
        """SDK2 回调线程：四足解算位姿 → nav_msgs/Odometry"""
        self.n += 1
        if self.n % 25 == 1:  # 50Hz 里抽一条打日志
            q = m.imu_state_.quaternion_
            self.get_logger().info(
                'pos=(%.3f,%.3f,%.3f) vel=(%.2f,%.2f) yaw_v=%.2f quat=[%.3f %.3f %.3f %.3f] mode=%d gait=%d'
                % (m.position_[0], m.position_[1], m.position_[2],
                   m.velocity_[0], m.velocity_[1], m.yaw_speed_,
                   q[0], q[1], q[2], q[3], m.mode_, m.gait_type_))

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'go2_odom'
        odom.child_frame_id = 'go2_base'
        odom.pose.pose.position.x = float(m.position_[0])
        odom.pose.pose.position.y = float(m.position_[1])
        odom.pose.pose.position.z = float(m.position_[2])
        q = m.imu_state_.quaternion_  # [w,x,y,z]
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])
        odom.pose.pose.orientation.w = float(q[0])
        odom.twist.twist.linear.x = float(m.velocity_[0])
        odom.twist.twist.linear.y = float(m.velocity_[1])
        odom.twist.twist.linear.z = float(m.velocity_[2])
        odom.twist.twist.angular.z = float(m.yaw_speed_)
        self.pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'go2_odom'
        t.child_frame_id = 'go2_base'
        t.transform.translation.x = float(m.position_[0])
        t.transform.translation.y = float(m.position_[1])
        t.transform.translation.z = float(m.position_[2])
        t.transform.rotation = odom.pose.pose.orientation
        self.tfb.sendTransform(t)


def main():
    rclpy.init()
    node = Go2OdomBridge()
    ChannelFactoryInitialize(0, 'eth1')
    # 固件是 ROS2 程序，话题带 rt/ 前缀（rclpy 版订阅 /lf/sportmodestate 已实测有数据）
    sub = ChannelSubscriber('rt/lf/sportmodestate', SportModeState_)
    sub.Init(node.on_state)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
