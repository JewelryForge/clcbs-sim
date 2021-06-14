#!/usr/bin/env python3

import rospy
import tf
import argparse
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose


class GazeboLinkPose:
    link_name = ''
    link_pose: Pose = Pose()

    def __init__(self, robot_name, link_name):
        self.robot_name = robot_name
        self.link_name = link_name
        # self.link_name_rectified = link_name.replace("::", "_")

        if not self.link_name:
            raise ValueError("'link_name' is an empty string")

        self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)
        self.pose_pub = rospy.Publisher(
            "/gazebo/" + (self.robot_name + '_' + self.link_name),
            Pose,
            queue_size=10)
        self.tf_pub = tf.TransformBroadcaster()

    def callback(self, data):
        try:
            idx = data.name.index(self.robot_name + "::" + self.link_name)
            self.link_pose: Pose = data.pose[idx]
        except ValueError:
            pass

    def publish_tf(self):
        p, o = self.link_pose.position, self.link_pose.orientation
        translation = [p.x, p.y, p.z]
        rotation = [o.x, o.y, o.z, o.w]
        self.tf_pub.sendTransform(translation, rotation, rospy.Time.now(), self.link_name, "map")
        print('tf published')


if __name__ == '__main__':
    try:
        rospy.init_node('robot_pose_tf_publisher')
        parser = argparse.ArgumentParser(description='Receive transforms from gazebo and send tf')
        parser.add_argument('-r', type=str, required=True)
        parser.add_argument('-l', type=str, required=True)
        parser.add_argument('__name')
        parser.add_argument('__log')
        args = parser.parse_args()
        gp = GazeboLinkPose(args.r, args.l)
        publish_rate = rospy.get_param('publish_rate', 100)
        rate = rospy.Rate(publish_rate)
        while not rospy.is_shutdown():
            gp.pose_pub.publish(gp.link_pose)
            gp.publish_tf()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
