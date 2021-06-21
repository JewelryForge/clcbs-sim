rostopic pub /agent0/left_wheel_controller/command std_msgs/Float64 "data: 0.0" -1 & \
rostopic pub /agent1/left_wheel_controller/command std_msgs/Float64 "data: 0.0" -1 & \
rostopic pub /agent2/left_wheel_controller/command std_msgs/Float64 "data: 0.0" -1 & \
rostopic pub /agent3/left_wheel_controller/command std_msgs/Float64 "data: 0.0" -1 & \
rostopic pub /agent4/left_wheel_controller/command std_msgs/Float64 "data: 0.0" -1 & \
rostopic pub /agent0/right_wheel_controller/command std_msgs/Float64 "data: 0.0" -1 & \
rostopic pub /agent1/right_wheel_controller/command std_msgs/Float64 "data: 0.0" -1 & \
rostopic pub /agent2/right_wheel_controller/command std_msgs/Float64 "data: 0.0" -1 & \
rostopic pub /agent3/right_wheel_controller/command std_msgs/Float64 "data: 0.0" -1 & \
rostopic pub /agent4/right_wheel_controller/command std_msgs/Float64 "data: 0.0" -1;
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'agent0'
  pose:
    position:
      x: -11.0
      y: 14.0
    orientation:
      w: 1.0
      z: 0.0";
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'agent1'
  pose:
    position:
      x: 2.0
      y: -23.0
    orientation:
      w: 0.707
      z: -0.707";
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'agent2'
  pose:
    position:
      x: 16.0
      y: 18.0
    orientation:
      w: 0.707
      z: 0.707";
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'agent3'
  pose:
    position:
      x: -13.0
      y: -6.0
    orientation:
      w: 0.0
      z: -1.0";
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'agent4'
  pose:
    position:
      x: -12.0
      y: 21.0
    orientation:
      w: 1.0
      z: 0.0";
      