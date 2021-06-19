#!/usr/bin/env python3

import rospy
import tf
import sys
from std_msgs.msg import Header
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, PoseStamped


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
            self.publishers[topic] = rospy.Publisher(f'/agent_states/{topic}', Pose, queue_size=10)

    def callback(self, data: LinkStates):
        try:
            for topic in self.publishers:
                robot, link = topic.split('/')
                idx = data.name.index(robot + "::" + link)
                self.publishers[topic].publish(data.pose[idx])
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
