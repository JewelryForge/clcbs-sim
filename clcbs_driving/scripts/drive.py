#!/usr/bin/env python3
import argparse
import os
import yaml
import math
import numpy as np
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import gazebo_msgs.srv as gazebo_srvs
import rospy
import PyKDL
import tf


class VelocityController:
    def __init__(self):
        self.vx, self.vw, self.step = 0.0, 0.0, 0.1
        self.vw_limit = 0.0
        self.vmax, self.wmax = 10.0, 3.0
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
    
    def set_vx(self, vx):
        if vx > self.vx + self.vmax * self.step:
            self.accelerate()
        elif vx < self.vx - self.vmax * self.step:
            self.decelerate()
        else:
            self.vx = vx
            self.vw_limit = min(abs(self.vx) / self.rot_radius, self.wmax)
            self.limit_all()

    def set_vw(self, vw):
        if vw > self.vw + self.step * self.vw_limit:
            self.left_turn()
        elif vw < self.vw - self.step * self.vw_limit:
            self.right_turn()
        else:
            self.vw = vw
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
        self.aligning_param = np.array([-25, -25, 0])

    def align(self, pos: np.ndarray):
        return pos + np.array(self.aligning_param)

    def get_state(self, t):
        return self.align(self._get_state(t))

    def _get_state(self, t):
        idx = int(t) + 1

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

def angle_norm(a):
    if a > 2 * math.pi:
        return angle_norm(a - 2 * math.pi)
    if a <= -2 * math.pi:
        return angle_norm(a + 2 * math.pi)
    return a

def angle_diff(a1, a2):
    return angle_norm(a1 - a2)
    
class PidVelocityPublisher(VelocityController):
    def __init__(self, name, states, **args):
        super().__init__()
        self.left_pub = rospy.Publisher('/agent_control/agent_leftwheel_controller/command',
                                        std_msgs.Float64, queue_size=1)
        self.right_pub = rospy.Publisher('/agent_control/agent_rightwheel_controller/command',
                                         std_msgs.Float64, queue_size=1)
        self.state_sub = rospy.Subscriber('/agent_states/agent1_robot_base', geometry_msgs.Pose,
                                          callback=self.state_update)
        self.tf_pub = tf.TransformBroadcaster()
        self.radius = (2 + 0.5 / 3) / 2
        self.prop, self.int, self.diff = args.get(
            'prop', 10), args.get('int', 1), args.get('diff, 0')
        self.t_start = None
        self.state_manager = StateManager(name, states)
        self.curr_state = None

    def start(self):
        self.t_start = rospy.get_time()

    def state_update(self, msg: geometry_msgs.Pose):
        rot = PyKDL.Rotation.Quaternion(msg.orientation.x, msg.orientation.y,
                                        msg.orientation.z, msg.orientation.w)

        self.curr_state = np.array(
            [msg.position.x, msg.position.y, rot.GetRPY()[2]])

    def spin(self):
        rate = rospy.Rate(20)
        try:
            while not rospy.is_shutdown():
                if self.curr_state is not None:
                    if self.t_start is None:
                        self.start()
                    self.pub(True)
                rate.sleep()
        except KeyboardInterrupt:
            os.system('./reset.py')
        


    def pub(self, verbose=True):
        diff_time = rospy.get_time() - self.t_start
        desired_state = self.state_manager.get_state(diff_time)
        des_x, des_y, des_yaw = desired_state
        self.tf_pub.sendTransform([des_x, des_y, 0],  PyKDL.Rotation.RotZ(des_yaw).GetQuaternion(),
                                  rospy.Time.now(), "desired_state", "map")

        dx, dy, dyaw = (self.state_manager.get_state(diff_time + 0.1) - self.state_manager.get_state(diff_time)) / 0.1
        dv = dx / math.cos(desired_state[2])
        # print('desired', dx, dy, dyaw, end='')
        diff_state = desired_state - self.curr_state
        yaw = self.curr_state[2]
        self.set_vx(2 * (diff_state[0] * math.cos(yaw) + diff_state[1] * math.sin(yaw))) # 反馈控制法
        self.set_vw(angle_diff(desired_state[2], self.curr_state[2]))
        
        self.left_pub.publish(self.vx - self.vw * self.radius)
        self.right_pub.publish(self.vx + self.vw * self.radius)
        if verbose:
            print(f'[{rospy.get_time()}]', diff_state, f"publish: {self.vx:.1f} {self.vw:.1f}")


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
    pub.spin()
    # rospy.spin()
    # managers = []
    # for name, sch in schedule.items():

    # m: StateManager = managers[0]
