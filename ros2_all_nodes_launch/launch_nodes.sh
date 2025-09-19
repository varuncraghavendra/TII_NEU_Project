GNU nano 6.2                                                  launch_ros2_nodes.sh                                                            
#!/bin/bash
# filename: launch_all.sh

# Start a new tmux session named "drone_system"
tmux new-session -d -s drone_system

# Window 1: Launch basic drone control nodes
tmux rename-window -t drone_system:0 'drone_control'
tmux send-keys -t drone_system:0 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t drone_system:0 'ros2 launch drone_basic_control launch_all_nodes.launch.py' C-m

# Window 2: Run goal manager server
tmux new-window -t drone_system:1 -n 'goal_manager'
tmux send-keys -t drone_system:1 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t drone_system:1 'ros2 run goal_manager goal_manager_server' C-m

# Window 3: Launch system bringup nodes
tmux new-window -t drone_system:2 -n 'system_bringup'
tmux send-keys -t drone_system:2 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t drone_system:2 'ros2 launch system_bringup launch_nodes.launch.py' C-m

# Attach to the tmux session
tmux attach -t drone_system

