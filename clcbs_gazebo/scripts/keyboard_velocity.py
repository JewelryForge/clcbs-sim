#!/usr/bin/env python3

import std_msgs.msg as std_msgs
import rospy
from pynput.keyboard import Key, Listener


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



class VelocityPublisher(VelocityController):
    def __init__(self):
        super().__init__()
        self.left_pub = rospy.Publisher('/agent_control/agent_leftwheel_controller/command',
                                        std_msgs.Float64, queue_size=1)
        self.right_pub = rospy.Publisher('/agent_control/agent_rightwheel_controller/command',
                                         std_msgs.Float64, queue_size=1)
        self.radius = (2 + 0.5 / 3) / 2
        self.KBD_MAP = {
            'w': self.accelerate,
            's': self.decelerate,
            'a': self.left_turn,
            'd': self.right_turn,
            Key.space: self.reset
        }

    def pub(self, verbose=True):
        self.left_pub.publish(self.vx - self.vw * self.radius)
        self.right_pub.publish(self.vx + self.vw * self.radius)

        if rospy.is_shutdown():
            exit()
        if verbose:
            print(f"publish: left - {self.vx - self.vw * self.radius:.2f}"
                  f"right - {self.vx + self.vw * self.radius:.2f}")

    def kbd_input(self, key):
        def nop():
            pass
        self.KBD_MAP.get(key, nop)()
        self.pub()


def press_function(publisher: VelocityPublisher):
    def on_press(key):
        convert_key = key.char if hasattr(key, 'char') else key
        publisher.kbd_input(convert_key)
    return on_press


def main():
    rospy.init_node("velocity_publisher", anonymous=True)
    publisher = VelocityPublisher()
    with Listener(on_press=press_function(publisher)) as listener:
        listener.join()


if __name__ == '__main__':
    main()
