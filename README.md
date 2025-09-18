# TII_NEU_Project

Launch the DDS Agent on a new terminal : 

`MicroXRCEAgent udp4 -p 8888`

### Enter PX4 Docker using the following commands : 

`docker start px4-dev-test`

`docker exec -it px4-dev-test bash`

NOTE : Enter the PX4 directory before running commands 

`cd PX4-Autopilot`

### To Spawn only one drone and run the simulation, run the following command : 

`PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4`

### To Spawn Multiple Drones, run these in different terminals within the docker : 

`PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0" PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 1`

`PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 2`

`PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,2" PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 3`

### Enter ROS2 Docker using the following commands : 

`docker start px4_agent_ws`

`docker exec -it px4_agent_ws bash`

#### Run the following in new terminals inside the ROS2 Docker :

`ros2 launch drone_basic_control launch_all_nodes.launch.py`

`ros2 run goal_manager goal_manager_server`

`ros2 launch system_bringup launch_nodes.launch.py `


### Square Pattern Example 

Outside the Docker, run the following commands in 2 separate terminals 

`cd drone_pattern`

`python3 square_pattern.py`

`cd mqtt_client`

`python3 mqtt_client.py`
