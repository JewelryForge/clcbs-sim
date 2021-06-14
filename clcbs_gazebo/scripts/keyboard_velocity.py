#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import std_msgs.msg as std_msgs
import rospy
from pynput.keyboard import Key, Listener

command = {"vx": 0.0, "vw": 0.0, "step": 0.1, "vmax": 3.0}


def limitNum(num, minNum, maxNum):
    return maxNum if num > maxNum else max(num, minNum)

def cmd_num_func(cmd, key, value, minValue, maxValue):
    def function():
        cmd[key] = limitNum(cmd[key] + value, minValue, maxValue)

    return function

def cmd_reset_func(cmd):
    def function():
        cmd["vx"], cmd["vw"]  = 0.0, 0.0

    return function


KEY_MAP_TABLE = {
    'w': cmd_num_func(command, "vx", command["step"], -command["vmax"], command["vmax"]),
    's': cmd_num_func(command, "vx", -command["step"], -command["vmax"], command["vmax"]),
    'a': cmd_num_func(command, "vw", command["step"], -command["vmax"], command["vmax"]),
    'd': cmd_num_func(command, "vw", -command["step"], -command["vmax"], command["vmax"]),
    Key.space: cmd_reset_func(command)
}

class VelocityPublisher:
    def __init__(self):
        self.left_pub = rospy.Publisher('/agent_control/agent_leftwheel_controller/command',
                                        std_msgs.Float64, queue_size=1)
        self.right_pub = rospy.Publisher('/agent_control/agent_rightwheel_controller/command',
                                         std_msgs.Float64, queue_size=1)
        self.radius = (2 + 0.5 / 3) / 2

    def pub(self, command):
        vx, vw = command['vx'], command['vw']
        self.left_pub.publish(vx - vw  * self.radius)
        self.right_pub.publish(vx + vw  * self.radius)

        if rospy.is_shutdown():
            exit()
        print(f"publish command : vx - {command['vx']:.2f} vw - { command['vw']:.2f}")


def press_function(map_table, publisher):
    def on_press(key):
        global NEED_EXIT
        if key == Key.esc:
            NEED_EXIT = True
            return False
        try:
            convert_key = key.char
        except AttributeError:
            convert_key = key
        if convert_key in map_table:
            map_table[convert_key]()
            publisher.pub(command)
        else:
            print('\n{0} not in table'.format(convert_key))

    return on_press


def main():
    node_name = "velocity_publisher"
    print("node : ", node_name)
    rospy.init_node(node_name, anonymous=True)
    publisher = VelocityPublisher()
    with Listener(on_press=press_function(KEY_MAP_TABLE, publisher)) as listener:
        listener.join()


if __name__ == '__main__':
    main()
