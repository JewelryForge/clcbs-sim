#!/bin/zsh

dst=$(rospack find clcbs_gazebo)
rosrun map_from_yaml map_from_yaml.py -i "$1" -o "$dst/models/heightmap/map/map.png" -c "$dst/models/heightmap/map.yaml"
rosrun clcbs_gazebo generate_launch.py -m "$1" -s "$2"
