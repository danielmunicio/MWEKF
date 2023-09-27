# Welcome to the FEB (soon to be) official GitHub Repo!

## Installation
Here, we will cover everything you will need to get going with coding for FEB Autonomous.


### Recommended environment
* Ubuntu 22.04 Jammy Jellyfish (TODO: Add internal link to installations for different OS)
* [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)

### SSH keys
To be able to push your code to our repo, you will need to generate an SSH key.
``` bash
ssh-keygen -t ed25519 -C "your_email@example.com"
```
The `"your_email@example.com"` can be replaced with any identifier you see fit. You will then be presented with some options and default shold work fine; press `ENTER` to accept the default settings.

Copy the contents of
``` bash
cat ~/.ssh/id_ed25519.pub
```
and add the key under `Settings > SSH and GPG keys` on GitHub.

### Clone repo to your machine
``` bash
cd ~/
git clone git@github.com:FEBAutonomous/feb-system-integration.git
```

### RoboSense M1 LiDAR ROS2 driver
