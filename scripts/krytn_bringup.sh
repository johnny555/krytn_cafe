#!/bin/bash
source /home/ubuntu/krytn_ws/install/setup.bash
ddsrouter -c /home/ubuntu/krytn_ws/src/krytn_cafe/config/dds_router_krytn.yaml & 
ros2 launch krytn_cafe krytn_bringup_launch.py  
