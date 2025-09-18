# TII_NEU_Project

# Steps to run the code : 
Launch the XRCE Agent on a new terminal : 
`MicroXRCEAgent udp4 -p 8888`

# Enter ROS2 Docker using the following commands : 
`docker start px4_agent_ws`
`docker exec -it px4_agent_ws bash`

### Run the following in new terminals inside the ROS2 Docker :
`ros2 launch drone_basic_control launch_all_nodes.launch.py`
`ros2 run goal_manager goal_manager_server`
`ros2 launch system_bringup launch_nodes.launch.py `

# To Spawn Multiple Drones, run these in different terminals : 
`PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0" PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 1`
`PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 2`
`PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,2" PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 3`



