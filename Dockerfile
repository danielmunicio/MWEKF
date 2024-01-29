# Build while inside repo
# docker build -t feb_image .

# Run from root directory
# docker run -it --user leonid -v bag_files:/home/leonid/bag_files /dev:/dev --device-cgroup-rule='c *:* rmw' feb_image


FROM osrf/ros:humble-desktop-full

ARG USERNAME=leonid
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid  $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config \
    && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Switch to repo and copy requirements FIRST to utilize cache
WORKDIR /home/$USERNAME
COPY ./requirements feb-system-integration/requirements

# Apt installs and set up sudo
RUN apt-get update \
  && xargs apt-get -y install < feb-system-integration/requirements/apt.txt\
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Pip installs
RUN pip3 install --no-cache-dir -r feb-system-integration/requirements/pip.txt

# Copy repo here! Use cache for everything above
COPY . feb-system-integration

# Source and build
RUN . /opt/ros/humble/setup.sh \
  && cd feb-system-integration && colcon build



