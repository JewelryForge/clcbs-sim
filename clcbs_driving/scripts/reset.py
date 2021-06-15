#!/usr/bin/env python3

import numpy as np
import math
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import gazebo_msgs.srv as gazebo_srvs
import rospy
import PyKDL

class IsStatic:
    def __init__(self) -> None:
        self.prev_state, self.curr_state = None, None

    


if __name__ == "__main__":
    rospy.init_node('reset')
    prev_state, curr_state = None, None
    def state_update(msg):
        global curr_state
        rot = PyKDL.Rotation.Quaternion(msg.orientation.x, msg.orientation.y,
                                        msg.orientation.z, msg.orientation.w)

        curr_state = np.array(
            [msg.position.x, msg.position.y, rot.GetRPY()[2]])
    left_pub = rospy.Publisher('/agent_control/agent_leftwheel_controller/command',
                                std_msgs.Float64, queue_size=1)
    right_pub = rospy.Publisher('/agent_control/agent_rightwheel_controller/command',
                                std_msgs.Float64, queue_size=1)
    state_sub = rospy.Subscriber('/agent_states/agent1_robot_base', geometry_msgs.Pose,
                                 callback=state_update)
    rate = rospy.Rate(20)
    while prev_state is None or math.hypot(*(curr_state - prev_state)) > 1e-3:
        prev_state = curr_state
        left_pub.publish(0.0)
        right_pub.publish(0.0)
        rate.sleep()

    rospy.wait_for_service('/gazebo/set_model_state')
    reset = rospy.ServiceProxy('/gazebo/set_model_state', gazebo_srvs.SetModelState)
    init_state = gazebo_srvs.SetModelStateRequest()
    init_state.model_state.model_name = 'agent1'
    init_state.model_state.pose.position.x = -11.0
    init_state.model_state.pose.position.y = 14.0
    reset(init_state)
    print('reset')