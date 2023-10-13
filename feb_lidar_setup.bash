#!/bin/bash

# Identify the Ethernet interface
ethernet_interface=$(ip -o link show | awk -F: '$2 !~ /lo|vir/ && $2 ~ /en/{gsub(/^[ \t]+|[ \t]+$/, "", $2); print $2; exit}')

# Check if an Ethernet interface was found
if [ -z "$ethernet_interface" ]; then
  echo "No Ethernet interface found."
  exit 1
fi

# Define the connection details
connection_name="RS M1 LiDAR"
ip_address="192.168.102.150/24"
msop="6702"

# Check if the connection profile already exists
if nmcli con show "$connection_name" &> /dev/null; then
  echo "Connection profile '$connection_name' already exists."
else
  # Create the connection profile
  nmcli con add type ethernet ifname "$ethernet_interface" con-name "$connection_name"
  nmcli con modify "$connection_name" ipv4.method manual ipv4.address "$ip_address"
  nmcli con up "$connection_name"
fi

# Make sure firewall does not block the packets
echo "Password needed to modify firewall rules"

# Check if the firewall rule exists for port MSOP/UDP
if ! sudo ufw status numbered | grep -q "$msop/udp"; then
  # Add the firewall rule
  sudo ufw allow "$msop"/udp
  echo "Added firewall rule to allow UDP traffic on port $msop."
else
  echo "Firewall rule for port $msop/udp already exists."
fi

