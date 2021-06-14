#!/usr/bin/env python3
import argparse
from os import name
from re import S
import yaml
import math
import numpy as np
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import rospy
import PyKDL


class VelocityController:
    def __init__(self):
        self.vx, self.vw, self.step = 0.0, 0.0, 0.1
        self.vw_limit = 0.0
        self.vmax, self.wmax = 3.0, 3.0
        self.rot_radius = 3.0

    def accelerate(self):
        self.vx += self.vmax * self.step
        self.vw_limit = min(abs(self.vx) / self.rot_radius, self.wmax)
        self.limit_all()

    def decelerate(self):
        self.vx -= self.vmax * self.step
        self.vw_limit = min(abs(self.vx) / self.rot_radius, self.wmax)
        self.limit_all()

    def left_turn(self):
        self.vw += self.step * self.vw_limit
        self.limit_all()

    def right_turn(self):
        self.vw -= self.step * self.vw_limit
        self.limit_all()

    def reset(self):
        self.vx, self.vw = 0.0, 0.0

    def limit_all(self):
        if self.vx > self.vmax:
            self.vx = self.vmax
        elif self.vx < -self.vmax:
            self.vx = -self.vmax
        if self.vw > self.vw_limit:
            self.vw = self.vw_limit
        elif self.vw < -self.vw_limit:
            self.vw = -self.vw_limit


class StateManager:
    def __init__(self, name, states) -> None:
        self.name, self.states = name, states
        self.aligning_param = np.array([-30, -30, 0])

    def align(self, pos: np.ndarray):
        return pos - np.array(self.aligning_param)

    def get_state(self, t):
        return self.align(self._get_state(t))

    def _get_state(self, t):
        idx = int(t)

        if idx == 0:
            start_state = self.states[0]
            return np.array([start_state['x'], start_state['y'], start_state['yaw']])

        elif idx < len(self.states):
            last_state, next_state = self.states[idx - 1], self.states[idx]
            last_yaw, next_yaw = last_state['yaw'], next_state['yaw']

            if (last_yaw - next_yaw) > math.pi:
                last_yaw = last_yaw - 2 * math.pi
            elif (next_yaw - last_yaw) > math.pi:
                last_yaw = last_yaw + 2 * math.pi

            last_pos = np.array([last_state['x'], last_state['y'], last_yaw])
            next_pos = np.array([next_state['x'], next_state['y'], next_yaw])

            dt = next_state['t'] - last_state['t']
            ratio = (t - last_state['t']) / dt
            interp = (next_pos - last_pos) * ratio + last_pos
            return interp
        else:
            final_state = self.states[-1]
            return np.array([final_state['x'], final_state['y'], final_state['yaw']])


class PidVelocityPublisher(VelocityController):
    def __init__(self, name, states, **args):
        super().__init__()
        self.left_pub = rospy.Publisher('/agent_control/agent_leftwheel_controller/command',
                                        std_msgs.Float64, queue_size=1)
        self.right_pub = rospy.Publisher('/agent_control/agent_rightwheel_controller/command',
                                         std_msgs.Float64, queue_size=1)
        self.state_sub = rospy.Subscriber('/agent_states/agent1_robot_base', geometry_msgs.Pose,
                                          callback=self.state_update)
        self.radius = (2 + 0.5 / 3) / 2
        self.prop, self.int, self.diff = args.get(
            'prop', 10), args.get('int', 1), args.get('diff, 0')
        self.t_start = None
        self.curr_state = None  # TODO: NOTE HERE!
        self.state_manager = StateManager(name, states)

    def start(self):
        self.t_start = rospy.get_time()

    def state_update(self, msg: geometry_msgs.Pose):
        rot = PyKDL.Rotation.Quaternion(msg.orientation.x, msg.orientation.y,
                                        msg.orientation.z, msg.orientation.w)

        self.curr_state = np.array(
            [msg.position.x, msg.position.y, rot.GetRPY()[2]])
        print(rospy.get_time() - self.t_start)

    def spin(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.curr_state is not None:
                self.pub(True)   
            rate.sleep()

    def pub(self, verbose=True):
        diff_time = rospy.get_time() - self.t_start
        diff_state = self.state_manager.get_state(diff_time) - self.curr_state
        self.vx = 1 * math.hypot(*diff_state[:2])
        self.vw = 1 * abs(diff_state[2] - self.curr_state[2])
        self.limit_all()
        self.left_pub.publish(self.vx - self.vw * self.radius)
        self.right_pub.publish(self.vx + self.vw * self.radius)
        if verbose:
            print(f"publish: vx - {self.vx:.2f} vw - {self.vw:.2f}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--map", default=None,
                        help="input file containing map")
    parser.add_argument("-s", "--schedule", default=None,
                        help="schedule for agents")
    args = parser.parse_args()

    with open(args.map) as map_file:
        map = yaml.load(map_file, Loader=yaml.FullLoader)

    with open(args.schedule) as states_file:
        schedule = yaml.load(
            states_file, Loader=yaml.FullLoader)['schedule']

    rospy.init_node('agent_drive')
    name = 'agent0'
    sch = schedule[name]
    pub = PidVelocityPublisher(name, sch)
    pub.start()
    pub.spin()
    # rospy.spin()
    # managers = []
    # for name, sch in schedule.items():

    # m: StateManager = managers[0]
