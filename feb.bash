#!/bin/bash

# Check if NetworkManager is installed
if ! command -v nmcli &>/dev/null; then
    echo "NetworkManager is not installed. Installing..."
    sudo apt-get update
    sudo apt-get install network-manager -y
    echo "NetworkManager has been installed. Please run this script again."
    set -e
fi

# Find the Wi-Fi interface name
wifi_interface=$(nmcli -t -f NAME c show --active | cut -d: -f2)
if [ -z "$wifi_interface" ]; then
    echo "Wi-Fi interface not found."
    set -e
fi

# Set the static IP address
new_ip_address="192.168.102.150"
subnet_mask="24"
gateway_ip="192.168.102.255"

# Check if the connection exists
if nmcli connection show --active | grep -q "$wifi_interface"; then
    echo "Connection for '$wifi_interface' already exists. Updating to LiDAR destination IP address..."
    nmcli connection modify "$wifi_interface" ipv4.method manual ipv4.addresses "$new_ip_address/$subnet_mask" ipv4.gateway "$gateway_ip"
    echo "IP address for '$wifi_interface' successfully updated!"
else
    echo "Creating a new connection profile for '$wifi_interface'..."
    nmcli connection add con-name "$wifi_interface" ifname "$wifi_interface" type wifi ssid "LiDARnetwork"
    nmcli connection modify "$wifi_interface" ipv4.method manual ipv4.addresses "$new_ip_address/$subnet_mask" ipv4.gateway "$gateway_ip"
    echo "IP address for '$wifi_interface' successfully updated!"
fi

# Restart NetworkManager to apply changes
echo "Restaring network manager..."
sudo systemctl restart NetworkManager

# Define the command to add to ~/.bashrc
command_to_add="source ~/feb-system-integration/lidar_ws/src/install/setup.bash"

# Check if the command is already in ~/.bashrc
if ! grep -qF "$command_to_add" ~/.bashrc; then
    # If not found, add it to ~/.bashrc
    echo "$command_to_add" >> ~/.bashrc
    echo "Commands added to ~/.bashrc. It will be sourced each time you open a terminal."
else
    # If already found, inform the user
    echo "Commands already exists in ~/.bashrc. No changes made."
fi
