#!/usr/bin/env python
import rospy
import tf2_ros
import sys
from std_msgs.msg import Header
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, TransformStamped


class GazeboLinkPose:
    def __init__(self, *argv):
        if not argv:
            raise ValueError("Need at least a topic name")
        self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)
        self.publishers = {}
        self.seq = 0
        for topic in argv:
            if topic[:2] == '__':
                continue
            topic.strip('/')
            self.publishers[topic] = rospy.Publisher('/agent_states/' + topic, Pose, queue_size=10)
        self.broadcaster = tf2_ros.TransformBroadcaster()

    def callback(self, data):
        try:
            for topic in self.publishers:
                robot, link = topic.split('/')
                idx = data.name.index(robot + "::" + link)
                p = data.pose[idx]
                self.publishers[topic].publish(p)
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "map"
                t.child_frame_id = topic
                t.transform.translation.x = data.pose[idx].position.x
                t.transform.translation.y = data.pose[idx].position.y
                t.transform.translation.z = data.pose[idx].position.z
                t.transform.rotation.w = data.pose[idx].orientation.w
                t.transform.rotation.x = data.pose[idx].orientation.x
                t.transform.rotation.y = data.pose[idx].orientation.y
                t.transform.rotation.z = data.pose[idx].orientation.z
                self.broadcaster.sendTransform(t)
            self.seq += 1 
        except ValueError:
            pass



if __name__ == '__main__':
    try:
        rospy.init_node('state_publisher')
        gp = GazeboLinkPose(*sys.argv[1:])
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
