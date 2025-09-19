#!/bin/bash

# A script to launch 10 PX4 SITL drones in a tmux session.
# IMPORTANT: Run this script from the root of your PX4-Autopilot directory.
# You MUST launch Gazebo in a separate terminal BEFORE running this script.

SESSION_NAME="multi_drone"
NUM_DRONES=10

# Kill any existing session with the same name to ensure a clean start
echo "Killing any existing tmux session named '$SESSION_NAME'..."
tmux kill-session -t $SESSION_NAME 2>/dev/null

echo "Creating new tmux session: $SESSION_NAME"

for i in $(seq 1 $NUM_DRONES); do
    WINDOW_NAME="Drone $i"
    POSE="0,$((i-1))"

    if [ $i -eq 1 ]; then
        # Create the first tmux session for Drone 1
        tmux new-session -d -s $SESSION_NAME -n "$WINDOW_NAME"
    else
        # Create a new window for each subsequent drone
        tmux new-window -t $SESSION_NAME -n "$WINDOW_NAME"
    fi

    echo "Launching $WINDOW_NAME at pose $POSE..."
    tmux send-keys -t $SESSION_NAME:"$WINDOW_NAME" \
    "PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"$POSE\" PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i $i" C-m

    echo "Waiting 5 seconds for $WINDOW_NAME to initialize..."
    sleep 5
done

# Attach to the tmux session
echo "Attaching to tmux session: $SESSION_NAME"
tmux attach-session -t $SESSION_NAME
