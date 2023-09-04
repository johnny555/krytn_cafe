#!/bin/bash
source /opt/ros/humble/local_setup.bash
source /home/krytn/krytn_ws/install/setup.bash
source /home/krytn/ddsrouter_ws/install/setup.bash
#ddsrouter -c /home/krytn/krytn_ws/src/krytn_cafe/config/dds_router_krytn.yaml & 
ros2 launch krytn_cafe perception.yaml &
ros2 launch krytn_cafe map_and_nav_launch.py  
