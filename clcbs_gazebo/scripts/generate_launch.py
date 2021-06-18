#!/usr/bin/env python3
import os
import argparse
import yaml
import math
import numpy as np
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import gazebo_msgs.srv as gazebo_srvs
import rospy
import PyKDL
import tf



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--map", help="input file containing map")
    parser.add_argument("-s", "--schedule", help="schedule for agents")
    args = parser.parse_args()
    config = None
    with open(args.map) as map_file:
        config = yaml.load(map_file, Loader=yaml.FullLoader)
    agents, map = config['agents'], config['map']
    dst = os.path.join(os.path.abspath('.'), '../launch/clcbs_config.launch')
    with open(dst, 'w') as of:
        of.writelines([
            '<launch>\n', 
            '    <arg name="map_file" default="{}" />\n'.format(os.path.abspath(args.map)),
            '    <group ns="config">\n',
            '      <rosparam file="$(arg map_file)" command="load" />\n',
            '    </group>\n'
        ])


        of.write(
            '</launch>\n'
        )
