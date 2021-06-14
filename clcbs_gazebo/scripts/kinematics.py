#!/usr/bin/env python3

import geometry_msgs.msg as geometry_msgs
import rospy
import std_msgs.msg as std_msgs


class KinematicsPublisher:
    def __init__(self):
        self.left_pub = rospy.Publisher('/course_agv/left_wheel_velocity_controller/command',
                                        std_msgs.Float64, queue_size=1)
        self.right_pub = rospy.Publisher('/course_agv/right_wheel_velocity_controller/command',
                                         std_msgs.Float64, queue_size=1)

        self.sub = rospy.Subscriber('/course_agv/velocity', geometry_msgs.Twist, callback=self.callback)
        self.radius = (0.2 + 0.08 / 3) / 2

    def callback(self, twist: geometry_msgs.Twist):
        v, w = twist.linear.x, twist.angular.z
        self.left_pub.publish(v - w  * self.radius)
        self.right_pub.publish(v + w  * self.radius)


def main():
    rospy.init_node("Kinematics", anonymous=True)
    publisher = KinematicsPublisher()
    rospy.spin()


if __name__ == '__main__':
    main()
