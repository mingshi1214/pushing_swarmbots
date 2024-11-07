# pushing_swarmbots
Pushing Swarm Robots

$ ros2 launch cpmr_ch2 gazebo_swarm.launch.py 

$ ros2 launch cpmr_ch2 build_map.launch.py 

$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=block_robot_{#}/cmd_vel
