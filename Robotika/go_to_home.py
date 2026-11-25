#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool

class GoToHomeNode(Node):
    def __init__(self):
        super().__init__('go_to_home_node')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/xarm7_traj_controller/follow_joint_trajectory')
        self.ready_pub = self.create_publisher(Bool, '/robot_ready', 10)
        self.timer = self.create_timer(2.0, self.send_goal)

    def send_goal(self):
        self.timer.cancel()
        self._action_client.wait_for_server()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        point = JointTrajectoryPoint(positions=[-0.687, -0.767, -1.352, 2.158, -0.920, 2.122, -2.562], time_from_start=Duration(sec=2))
        goal.trajectory.points = [point]
        self._action_client.send_goal_async(goal).add_done_callback(self.done_cb)

    def done_cb(self, future):
        self.ready_pub.publish(Bool(data=True))

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GoToHomeNode())
    rclpy.shutdown()
