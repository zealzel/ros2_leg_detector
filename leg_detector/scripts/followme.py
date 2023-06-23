#!/usr/bin/env python3

import time
from math import atan2, sqrt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import (
    TransformBroadcaster,
    TransformListener,
    Buffer,
)
from geometry_msgs.msg import TransformStamped, Twist
from leg_detector_msgs.msg import PersonArray


class FollowMeNode(Node):
    def __init__(self):
        super().__init__("follow_me")

        self.person_found = False
        self.t0 = time.time()

        self.transform_broadcaster = TransformBroadcaster(self)
        self.buffer = Buffer()
        self.transform_listener = TransformListener(self.buffer, self)

        qos = QoSProfile(depth=3)
        self.subscription = self.create_subscription(
            PersonArray, "/people_tracked", self.pose_callback, qos
        )
        self.publisher = self.create_publisher(Twist, "/cmd_vel", qos)

        # self.timer = self.create_timer(0.2, self.timer_callback)  # 5Hz
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        self.dx = 0.0
        self.dy = 0.0
        self.distance_between = 0.4
        self.K_linear = 0.2
        self.K_angular = 0.4
        self.person_id = -1

    def pose_callback(self, msg):
        if not msg.people:
            self.person_found = False
            self.person_id = -1
        else:
            self.person_found = True
            self.t0 = time.time()
            # person_id = people.id
            if len(msg.people) == 1:
                person = msg.people[0]
                if self.person_id != person.id:
                    self.person_id = person.id
                    print(f"person to track! person_id = {self.person_id}")
            else:
                print(
                    f"people more than one!!! try to track person_id={self.person_id}"
                )
                ids = [p.id for p in msg.people]
                # print(f'ids: {ids}')
                if self.person_id and self.person_id in ids:
                    idx = ids.index(self.person_id)
                    # print("idx", idx)
                    person = msg.people[idx]
            position = person.pose.position
            self.dx = position.x
            self.dy = position.y
            print(f"pose_callback, dx: {self.dx}, dy: {self.dy}")

    def timer_callback(self):
        if not self.person_found:
            return
        d = sqrt(self.dx**2 + self.dy**2)
        dt = time.time() - self.t0
        # print(f"dx: {self.dx}, dy: {self.dy}, d: {d}")

        cmd = Twist()
        if d < self.distance_between:
            print("\n\nREADCHED!!!\n\n")
            # goal reached
            linear = 0.0
            angular = 0.0
        elif dt > 1.0:
            # person not found
            linear = 0.0
            angular = 0.0
        else:
            # linear = 1.5 * d
            # angular = 4.0 * atan2(dy, dx)
            linear = self.K_linear * d
            angular = self.K_angular * atan2(self.dy, self.dx)
            # print(f"linear: {linear}")
            # print(f"angular: {angular}")
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FollowMeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
