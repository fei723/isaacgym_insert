#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import math
import time
import numpy as np


class JointStatePublisherDemo:
    def __init__(self):
        rospy.init_node('joint_state_publisher_demo', anonymous=True)
        self.publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.start_time = time.time()

        self.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
            'joint7',
            'Joint_finger1',
            'Joint_finger2'
        ]
        # self.joint_positions = [
        #     0.0,
        #     0.3975287999999999,
        #     0.0,
        #     1.1252255999999998,
        #     0.0,
        #     1.2206576,
        #     1.5531576000000005,
        #     0.0,
        #     0.0
        # ]

        self.joint_positions = [
            0.0,
            60 / 180 * math.pi,
            0.0,
            0,
            0.0,
            (30+90-22.4)/180 * np.pi,
            90/180 * np.pi,
            0.0,
            0.0
        ]

    def publish_joint_states(self):
        while not rospy.is_shutdown():
            # 计算关节角度，动态变化
            # elapsed_time = time.time() - self.start_time
            # self.joint_positions[0] = math.sin(elapsed_time)  # 关节1角度随时间变化
            # self.joint_positions[1] = math.cos(elapsed_time)  # 关节2角度随时间变化
            # self.joint_positions[2] = math.sin(elapsed_time / 2)  # 关节3角度变化

            # 构造 JointState 消息
            msg = JointState()
            msg.header.stamp = rospy.Time.now()  # 设置时间戳
            msg.name = self.joint_names
            msg.position = self.joint_positions

            # 发布消息
            self.publisher.publish(msg)
            rospy.loginfo(f'Published joint states: {msg.position}')

            # 控制发布频率
            self.rate.sleep()


def main():
    node = JointStatePublisherDemo()
    try:
        node.publish_joint_states()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
