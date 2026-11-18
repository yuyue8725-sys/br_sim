#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Nav2 端到端测试：设初始位姿 -> 发 NavigateToPose 目标 -> 监控。

本仿真的 /odom 直接来自 Gazebo 真实世界位姿，且保存地图时 map 原点已对齐
世界坐标（maps/robocon_map.yaml 的 origin≈[-5.56,-5.57]），所以 map 系坐标
可以直接用世界坐标。
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw2q(y):
    return (0.0, 0.0, math.sin(y / 2.0), math.cos(y / 2.0))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavTest(Node):
    def __init__(self):
        super().__init__('nav_test')
        self.ipub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cpub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.amcl = None
        self._last_fb = 0.0
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.cb_amcl, 10)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def cb_odom(self, m):
        self.odom = m

    def cb_amcl(self, m):
        self.amcl = m

    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)

    def pose(self):
        p = self.odom.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def publish_twist(self, vx, vy, wz, sec=1.0):
        """直接给底盘发速度（Nav2 空闲时用这个来触发 AMCL 更新）。"""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        t0 = time.time()
        while time.time() - t0 < sec:
            self.cpub.publish(msg)
            self.spin(0.1)

    def set_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        q = yaw2q(yaw)
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        cov = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]
        msg.pose.covariance = cov
        for _ in range(5):
            self.ipub.publish(msg)
            self.spin(0.2)

    def goto(self, x, y, yaw, tmax=180.0):
        if not self.ac.wait_for_server(timeout_sec=20.0):
            print('  FAIL: navigate_to_pose action server 未就绪')
            return None
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        q = yaw2q(yaw)
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]
        fut = self.ac.send_goal_async(goal, feedback_callback=self.fb)
        self.spin(2.0)
        if not fut.done():
            self.spin(3.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print('  FAIL: 目标被拒绝')
            return None
        t0 = time.time()
        res_fut = gh.get_result_async()
        while not res_fut.done() and time.time() - t0 < tmax:
            self.spin(0.2)
        if not res_fut.done():
            print('  超时 %.0fs，取消目标' % tmax)
            gh.cancel_goal_async()
            return None
        return res_fut.result().status

    def fb(self, msg):
        # 反馈频率很高（几十 Hz），每 2 s 只打印一次，避免日志刷屏
        now = time.time()
        if now - self._last_fb < 2.0:
            return
        self._last_fb = now
        x, y, th = self.pose()
        self.get_logger().info('  反馈: 剩余 %.2f m  当前(%.2f, %.2f, yaw=%.2f)'
                               % (msg.feedback.distance_remaining, x, y, th))


def main():
    rclpy.init()
    n = NavTest()
    for _ in range(100):
        n.spin(0.05)
        if n.odom is not None:
            break
    x0, y0, t0 = n.pose()
    print('机器人当前位姿(世界): x=%.2f y=%.2f yaw=%.2f' % (x0, y0, t0))

    print('-> 发布 /initialpose = (%.2f, %.2f, yaw=%.2f)' % (x0, y0, t0))
    n.set_initial_pose(x0, y0, t0)
    # AMCL 只在滤波器更新（位姿有变化）时才发 /amcl_pose，静止时一条都不发。
    # 所以先原地转 1.5 s 逼它更新一次，再最多等 6 s 收位姿。
    n.spin(0.5)
    if n.amcl is None:
        n.publish_twist(0.0, 0.0, 0.6, sec=1.5)     # 原地转，触发 AMCL 更新
        n.publish_twist(0.0, 0.0, 0.0, sec=0.5)     # 停下
        for _ in range(12):
            n.spin(0.5)
            if n.amcl is not None:
                break
    if n.amcl is not None:
        p = n.amcl.pose.pose
        print('   AMCL 输出位姿: (%.2f, %.2f, yaw=%.2f)  cov_xy=(%.3f, %.3f)'
              % (p.position.x, p.position.y, yaw_of(p.orientation),
                 math.sqrt(n.amcl.pose.covariance[0]),
                 math.sqrt(n.amcl.pose.covariance[7])))
    else:
        print('   警告: 没收到 /amcl_pose（AMCL 静止时不发布，不影响后续导航）')

    goals = [(0.5, 4.6, 0.0), (-4.5, 1.3, 1.57)]
    for (gx, gy, gyaw) in goals:
        print('-> 导航目标 (%.2f, %.2f, yaw=%.2f)' % (gx, gy, gyaw))
        st = n.goto(gx, gy, gyaw)
        x, y, th = n.pose()
        print('   结束状态=%s  实际到位 (%.2f, %.2f, yaw=%.2f)  误差=%.2f m'
              % (st, x, y, th, math.hypot(x - gx, y - gy)))
        if st is None or st != 4:      # 4 = STATUS_SUCCEEDED
            print('   目标未成功，停止后续测试')
            break

    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
