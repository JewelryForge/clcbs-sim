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
    map_size = map['dimensions']
    dst = os.path.join(os.path.abspath('.'), '../launch/clcbs_config.launch')
    with open(dst, 'w') as of:
        of.write(
            f'<launch>\n'
            f'   <arg name="map_file" default="$(find clcbs_gazebo)/models/agent/config/map_50by50_obst25_agents5_ex37.yaml" />\n'
            f'   <group ns="config">\n'
            f'      <rosparam file="$(arg map_file)" command="load" />\n'
            f'   </group>\n\n'
        )
        state_args = []
        for i, agent in enumerate(agents):
            state_args.append(f'agent{i}/robot_base')
            start_state = agent['start']
            x, y, yaw = start_state[0] - map_size[0] / 2, start_state[1] - map_size[1] / 2, start_state[2]
            of.write(
                f'   <arg name="ns_{i}" value="agent{i}" />\n'
                f'   <group ns="$(arg ns_{i})">\n'
                f'      <rosparam file="$(find clcbs_gazebo)/models/agent/config/gazebo_controller.yaml" command="load" />\n'
                f'      <param name="robot_description" command="$(find xacro)/xacro $(find clcbs_gazebo)/models/agent/urdf/agent_sim.xacro ns:=$(arg ns_{i})"/>\n'
                f'      <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" args="-urdf -model $(arg ns_{i}) -param robot_description -x {x} -y {y} -Y {yaw}" />\n'
                f'      <node name="controller_spawner" pkg="controller_manager" type="spawner" args="joint_state_controller left_wheel_controller right_wheel_controller" />\n'
                f'      <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />\n'
                f'      <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" >\n'
                f'         <param name="tf_prefix" value="$(arg ns_{i})" />\n'
                f'      </node>\n'
                f'   </group>\n\n'
            )

        of.write(
            f'   <node pkg="clcbs_gazebo" type="state_publisher.py" name="robot_base_tf_publisher" args="{" ".join(state_args)}" />\n'
            f'</launch>\n'
        )
