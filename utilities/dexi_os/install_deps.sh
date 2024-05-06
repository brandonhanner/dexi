#!/bin/bash

############################## set up the build processs ##############################
#do this so apt has a dns resolver
mkdir -p /run/systemd/resolve
echo 'nameserver 1.1.1.1' > /run/systemd/resolve/stub-resolv.conf
#######################################################################################

#################################### update the OS ####################################
apt-get update -y &&  apt-get upgrade -y
#######################################################################################

################################ update boot config ###################################

{
   echo ""
   echo '# General Purpose UART External'
   echo 'dtoverlay=uart3,ctsrts'
   echo ""
   echo '# Telem2 to FC'
   echo 'dtoverlay=uart4,ctsrts'
   echo ""
   echo '# Drive USB Hub nRESET high'
   echo 'gpio=26=op,dh'
   echo ""
   echo '# Drive FMU_RST_REQ low'
   echo 'gpio=25=op,dl'
   echo ""
   echo '# Drive USB_OTG_FS_VBUS high to enable FC USB'
   echo 'gpio=27=op,dh'
   echo ""
   echo '# Disable Gigabit Ethernet, it can only do 100Mbps'
   echo 'dtoverlay=cm4-disable-gigabit-ethernet'
   echo ""
   echo 'dtoverlay=imx708,cam0'
   echo 'dtoverlay=imx708,cam1'
   echo ""
   echo 'dtoverlay=gpio-fan,gpiopin=19'
   echo ""
} >> /boot/firmware/config.txt

#######################################################################################

########################### install bootstrapping packages ############################
apt-get install -y \
wget \
curl \
git \
openssh-server \
adduser \
whois
#######################################################################################

################################## create dexi user ###################################
DEXI_PASS=dexi
PASS_HASH=$(mkpasswd -m sha-512 $DEXI_PASS)
#adduser -D dexi --comment "" --disabled-password
useradd -m -p "$PASS_HASH" -s /bin/bash dexi
usermod -aG sudo dexi
#######################################################################################

###################################  install docker ###################################
curl --output /tmp/get-docker.sh https://get.docker.com
chmod +x /tmp/get-docker.sh
/tmp/get-docker.sh
groupadd docker
usermod -aG docker dexi
#######################################################################################

#################################### install ROS 2 ####################################
# Setup apt repos
apt install software-properties-common
add-apt-repository universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" |  tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt update

# Install ROS2 humble
apt install ros-humble-ros-base ros-dev-tools ros-humble-rosbridge-server -y
rosdep init
#######################################################################################

############################### provision runonce daemon ##############################
# creates a job that only runs once (AKA on first boot)
# we'll use this to provision the wifi stuff once the SD card is put into the PI

#create the dir structure
mkdir -p /etc/local/runonce.d/ran
cp /tmp/resources/runonce /usr/local/bin/runonce
chmod +x /usr/local/bin/runonce

#add the cron job
croncmd="/usr/local/bin/runonce"
cronjob="@reboot $croncmd"
( crontab -l | grep -v -F "$croncmd" ; echo "$cronjob" ) | crontab -
#######################################################################################

################################## install neofetch ###################################
apt-get install -y neofetch
echo 'neofetch' >> /home/dexi/.bashrc
#######################################################################################

################################### clone dexi repo ###################################
git clone https://github.com/DroneBlocks/dexi.git /home/dexi/dexi
# cd /home/dexi/dexi
# git checkout develop
#######################################################################################

#################################### clone ark repo ###################################
git clone https://github.com/ARK-Electronics/ark_companion_scripts.git /home/dexi/ark_companion_scripts
#######################################################################################

############################## setup wifi & hostapd stuff #############################
apt install -y iw wireless-tools

# testing.. will update next time i touch this...
cd /tmp/resources/raspiApWlanScripts || exit 1
chmod +x ./setup_wlan_and_AP_modes.sh
./setup_wlan_and_AP_modes.sh -D -s SuperFi -p XXXXXXXX -a dexi-45df -r droneblocks

#######################################################################################