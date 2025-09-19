#!/bin/bash

# A script to launch multiple PX4 SITL drones in a tmux session.
# IMPORTANT: Run this script from the root of your PX4-Autopilot directory.
# You MUST launch Gazebo in a separate terminal BEFORE running this script.

SESSION_NAME="multi_drone"

# Kill any existing session with the same name to ensure a clean start
echo "Killing any existing tmux session named '$SESSION_NAME'..."
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Create a new detached tmux session for the first drone
echo "Creating new tmux session: $SESSION_NAME"
tmux new-session -d -s $SESSION_NAME -n "Drone 1"

# --- Drone 1 ---
echo "Launching Drone 1..."
tmux send-keys -t $SESSION_NAME:"Drone 1" \
'PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0" PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 1' C-m

# Add a delay to give the drone time to initialize before starting the next one.
echo "Waiting 10 seconds for Drone 1 to initialize..."
sleep 10

# --- Drone 2 ---
echo "Launching Drone 2..."
tmux new-window -t $SESSION_NAME -n "Drone 2"
tmux send-keys -t $SESSION_NAME:"Drone 2" \
'PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 2' C-m

# Add another delay
echo "Waiting 10 seconds for Drone 2 to initialize..."
sleep 10

# Attach to the tmux session
echo "Attaching to tmux session: $SESSION_NAME"
tmux attach-session -t $SESSION_NAME
