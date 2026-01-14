#!/usr/bin/env python3

import math
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.parameter import Parameter

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class NearestRobotPID(Node):

    def __init__(self):
        super().__init__('nearest_robot_pid')

        # ================= SIM TIME =================
        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])

        # ================= CALLBACK GROUPS =================
        self.odom_cb_group = ReentrantCallbackGroup()
        self.timer_cb_group = ReentrantCallbackGroup()

        # ================= QOS =================
        self.odom_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # ================= ROBOT LIST =================
        self.robot_names = [f"tb{i}_{j}" for i in range(5) for j in range(5)]
        self.robot_count = len(self.robot_names)

        # ================= GOAL =================
        self.goal_x = 2.0
        self.goal_y = -1.5
        self.goal_tolerance = 0.15

        # ================= GAINS =================
        self.Kp_dist = 0.6
        self.Kp_yaw = 1.5

        # ================= STATE =================
        self.robot_poses = {}
        self.cmd_pubs = {}
        self.odom_subs = []

        self.robots_ready = False     # 🔥 readiness latch
        self.active_robot = None
        self.goal_reached = False

        # ================= SUBSCRIPTIONS =================
        for robot in self.robot_names:
            self.cmd_pubs[robot] = None
            sub = self.create_subscription(
                Odometry,
                f"/{robot}/odom",
                partial(self.odom_callback, robot=robot),
                self.odom_qos,
                callback_group=self.odom_cb_group
            )
            self.odom_subs.append(sub)

        # ================= TIMER =================
        self.timer = self.create_timer(
            0.2,
            self.control_loop,
            callback_group=self.timer_cb_group
        )

        self.get_logger().info("🚀 NearestRobotPID started")
        self.get_logger().info("⏳ Waiting for ALL 25 robots before selecting")

    # ================= ODOM CALLBACK =================
    def odom_callback(self, msg: Odometry, robot: str):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.robot_poses[robot] = (x, y, yaw)

    # ================= CONTROL LOOP =================
    def control_loop(self):

        # ---------- HARD WAIT ----------
        if not self.robots_ready:
            count = len(self.robot_poses)

            if count < self.robot_count:
                self.get_logger().warn(
                    f"⏳ Waiting for robot poses: {count}/{self.robot_count}",
                    throttle_duration_sec=2.0
                )
                return

            # 🔒 LATCH READY STATE
            self.robots_ready = True
            self.get_logger().info(
                f"✅ All {self.robot_count} robots are READY"
            )
            return   # 🔥 DO NOT SELECT IN SAME CYCLE

        # ---------- SELECT ACTIVE ROBOT ----------
        if self.active_robot is None:
            self.active_robot = min(
                self.robot_poses.items(),
                key=lambda r: math.hypot(
                    self.goal_x - r[1][0],
                    self.goal_y - r[1][1]
                )
            )[0]

            self.get_logger().info(
                f"🤖 Selected active robot: {self.active_robot}"
            )

        # ---------- STOP OTHERS ----------
        for robot, pub in self.cmd_pubs.items():
            if robot != self.active_robot and pub is not None:
                pub.publish(Twist())

        # ---------- CONTROL ----------
        if self.goal_reached:
            if self.cmd_pubs[self.active_robot]:
                self.cmd_pubs[self.active_robot].publish(Twist())
            return

        x, y, yaw = self.robot_poses[self.active_robot]

        dx = self.goal_x - x
        dy = self.goal_y - y

        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_error = math.atan2(
            math.sin(target_yaw - yaw),
            math.cos(target_yaw - yaw)
        )

        cmd = Twist()

        if dist > self.goal_tolerance:
            cmd.linear.x = min(self.Kp_dist * dist, 0.3)
            cmd.angular.z = max(min(self.Kp_yaw * yaw_error, 1.0), -1.0)
        else:
            self.goal_reached = True
            self.get_logger().info(
                f"🎯 Goal reached by {self.active_robot}"
            )

        if self.cmd_pubs[self.active_robot] is None:
            self.cmd_pubs[self.active_robot] = self.create_publisher(
                Twist,
                f"/{self.active_robot}/cmd_vel",
                10
            )

        self.cmd_pubs[self.active_robot].publish(cmd)


# ================= MAIN =================
def main():
    rclpy.init()
    node = NearestRobotPID()

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

