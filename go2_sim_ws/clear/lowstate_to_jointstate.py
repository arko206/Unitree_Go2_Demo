#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from unitree_go.msg import LowState


class LowStateToJointState(Node):
    def __init__(self):
        super().__init__('lowstate_to_jointstate')

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(
            LowState,
            '/lowstate',
            self.callback,
            10
        )

        # Trial mapping: EDIT THIS if leg order looks wrong in RViz

        ##---First Mapping---##
        # self.joint_names = [
        #     'LF_hip_joint',
        #     'LF_upper_leg_joint',
        #     'LF_lower_leg_joint',
        #     'RF_hip_joint',
        #     'RF_upper_leg_joint',
        #     'RF_lower_leg_joint',
        #     'LR_hip_joint',
        #     'LR_upper_leg_joint',
        #     'LR_lower_leg_joint',
        #     'RR_hip_joint',
        #     'RR_upper_leg_joint',
        #     'RR_lower_leg_joint',
        # ]

        ##---Second Mapping---##

        # self.joint_names = [
        #     'RF_hip_joint',
        #     'RF_upper_leg_joint',
        #     'RF_lower_leg_joint',
        #     'LF_hip_joint',
        #     'LF_upper_leg_joint',
        #     'LF_lower_leg_joint',
        #     'RR_hip_joint',
        #     'RR_upper_leg_joint',
        #     'RR_lower_leg_joint',
        #     'LR_hip_joint',
        #     'LR_upper_leg_joint',
        #     'LR_lower_leg_joint',
        # ]

        ##---Third Mapping---##
        self.joint_names = [
            'FR_hip_joint',
            'FR_thigh_joint',
            'FR_calf_joint',
            'FL_hip_joint',
            'FL_thigh_joint',
            'FL_calf_joint',
            'RR_hip_joint',
            'RR_thigh_joint',
            'RR_calf_joint',
            'RL_hip_joint',
            'RL_thigh_joint',
            'RL_calf_joint',
        ]    



        self.get_logger().info('LowState -> JointState bridge started')

    def callback(self, msg: LowState):
        if len(msg.motor_state) < 12:
            self.get_logger().warn('motor_state has fewer than 12 elements')
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = [msg.motor_state[i].q for i in range(12)]
        js.velocity = [msg.motor_state[i].dq for i in range(12)]
        js.effort = [msg.motor_state[i].tau_est for i in range(12)]

        self.pub.publish(js)


def main():
    rclpy.init()
    node = LowStateToJointState()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()