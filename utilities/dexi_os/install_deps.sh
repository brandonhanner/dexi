#!/bin/bash

#do this so apt has a dns resolver
mkdir -p /run/systemd/resolve
echo 'nameserver 1.1.1.1' > /run/systemd/resolve/stub-resolv.conf


sudo apt-get update -y && sudo apt-get upgrade -y
sudo apt-get install -y net-tools nano wget curl git iw wireless-tools

curl --output /tmp/get-docker.sh https://get.docker.com
chmod +x /tmp/get-docker.sh
sudo /tmp/get-docker.sh

# Setup apt repos
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

# Install ROS2 humble
sudo apt install ros-humble-ros-base ros-dev-tools ros-humble-rosbridge-server -y
sudo rosdep init