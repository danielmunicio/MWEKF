#!/bin/bash

# Define the command to add to ~/.bashrc
command_to_add="source ~/feb-system-integration/lidar_ws/install/setup.bash"

# Check if the command is already in ~/.bashrc
if ! grep -qF "$command_to_add" ~/.bashrc; then
    # If not found, add it to ~/.bashrc
    echo "$command_to_add" >> ~/.bashrc
    echo "Commands added to ~/.bashrc. It will be sourced each time you open a terminal."
else
    # If already found, inform the user
    echo "Commands already exists in ~/.bashrc. No changes made."
fi
