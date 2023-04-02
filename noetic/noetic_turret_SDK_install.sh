sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt-get install ros-noetic-depthai "ros-noetic-depthai*" "ros-noetic-dynamixel*" ros-noetic-desktop-full
sudo apt-get install python3-catkin python3-catkin-pkg-modules python3-rosdep python3-rosdep-modules python3-catkin-tools
sudo rosdep init
rosdep update

mkdir robotis_turret_ws/src -p
cd robotis_turret_ws/src
git clone https://github.com/MAVProxyUser/interbotix_ros_arms.git
git clone https://github.com/MAVProxyUser/interbotix_ros_turrets.git -b noetic
git clone https://github.com/MAVProxyUser/interbotix_ros_core.git
git clone https://github.com/MAVProxyUser/interbotix_ros_toolboxes.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git -b noetic-devel

cp -rfv interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk .
cp -rfv interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_msgs .
cp -rfv interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules .

cp -rfv interbotix_ros_arms/interbotix_examples/interbotix_moveit_interface .
cp -rfv interbotix_ros_arms/interbotix_moveit .
cp -rfv interbotix_ros_arms/interbotix_sdk .
cp -rfv interbotix_ros_arms/interbotix_descriptions .
rm -rf interbotix_ros_arms

cp interbotix_descriptions/urdf/vxxms.urdf.xacro interbotix_ros_turrets/interbotix_ros_xsturrets/interbotix_xsturret_descriptions/urdf/

source /opt/ros/noetic/setup.sh
cd ~/robotis_turret_ws/
rosdep install --from-paths src --ignore-src -r -y
catkin build
