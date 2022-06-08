set -e

sudo apt-get install --no-install-recommends -y git python3-pip
python3 -m pip install --no-cache catkin-tools
sudo apt-get install --no-install-recommends -y ros-${ROS_DISTRO}-tf2-sensor-msgs

