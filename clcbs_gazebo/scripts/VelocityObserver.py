#!/usr/bin/env python3

import math
import numpy as np
import geometry_msgs.msg as geometry_msgs
import rospy
import PyKDL

    
class VelocityObserver:
    def __init__(self):
        self.state_sub = rospy.Subscriber('/agent_states/agent1_robot_base', geometry_msgs.Pose,
                                          callback=self.state_update)
        self.last_time, self.last_state = None, None
        self.curr_state = None

    def state_update(self, msg: geometry_msgs.Pose):
        rot = PyKDL.Rotation.Quaternion(msg.orientation.x, msg.orientation.y,
                                        msg.orientation.z, msg.orientation.w)

        self.curr_state = np.array(
            [msg.position.x, msg.position.y, rot.GetRPY()[2]])

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            curr_time = rospy.get_time()
            if self.last_time is not None and self.last_state is not None:
                print((self.curr_state - self.last_state) / (curr_time - self.last_time))
            self.last_time = curr_time
            self.last_state = self.curr_state
            rate.sleep()



if __name__ == "__main__":
    rospy.init_node('agent_observer')
    ob = VelocityObserver()
    ob.spin()
