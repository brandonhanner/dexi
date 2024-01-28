docker run -it -v ${PWD}:/root/ros2_ws/src osrf/ros:humble-desktop

# Update repos

sudo apt update
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

# Create the dexi package

ros2 pkg create dexi --dependencies rclcpp --build-type ament_cmake

# A bridge from web into ROS

sudo apt install ros-humble-rosbridge-server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Camera

cd /root/ros2_ws/src
git clone https://github.com/Kapernikov/cv_camera
cd /root/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
ros2 run cv_camera cv_camera_node

# Web video server

cd /root/ros2_ws/src
git clone https://github.com/RobotWebTools/web_video_server/
cd web_video_server
git checkout ros2
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build

# LED support
