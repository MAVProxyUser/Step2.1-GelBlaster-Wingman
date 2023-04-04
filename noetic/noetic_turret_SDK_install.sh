#sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#sudo apt-get install curl nano
#sudo apt-get install guvcview
#curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
#sudo apt update
#sudo apt-get install ros-noetic-depthai "ros-noetic-depthai*" "ros-noetic-dynamixel*" ros-noetic-desktop-full
#sudo apt-get install python3-catkin python3-catkin-pkg-modules python3-rosdep python3-rosdep-modules python3-catkin-tools
#sudo rosdep init
#rosdep update

mkdir robotis_turret_ws/src -p
cd robotis_turret_ws/src
git clone https://github.com/MAVProxyUser/interbotix_ros_arms.git
git clone https://github.com/MAVProxyUser/interbotix_ros_turrets.git -b noetic
git clone https://github.com/MAVProxyUser/interbotix_ros_core.git
git clone https://github.com/MAVProxyUser/interbotix_ros_toolboxes.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git -b noetic-devel
git clone https://github.com/rbonghi/ros_jetson_stats.git

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

wget https://github.com/MAVProxyUser/Gelblaster_Wingman/raw/main/CC_BYNCSA_STL_Files/CC_BYNCSA_SurgeXL_quadruped_kit.stl
mv CC_BYNCSA_SurgeXL_quadruped_kit.stl src/interbotix_descriptions/meshes/meshes_vxxms/
wget https://github.com/MAVProxyUser/Step2.1-GelBlaster-Wingman/raw/main/noetic/turret_STL_replace.diff
cat turret_STL_replace.diff | patch -p0
cp ./src/interbotix_descriptions/urdf/vxxms.urdf.xacro ./src/interbotix_ros_turrets/interbotix_ros_xsturrets/interbotix_xsturret_descriptions/urdf/vxxms.urdf.xacro
cat ./src/interbotix_ros_xsturrets/interbotix_xsturret_gazebo/launch/xsturret_gazebo.launch | grep -v rviz_frame > ./src/interbotix_ros_xsturrets/interbotix_xsturret_gazebo/launch/xsturret_gazebo.launch.new
mv ./src/interbotix_ros_xsturrets/interbotix_xsturret_gazebo/launch/xsturret_gazebo.launch.new ./src/interbotix_ros_xsturrets/interbotix_xsturret_gazebo/launch/xsturret_gazebo.launch

catkin_make

wget https://github.com/MAVProxyUser/Step2.1-GelBlaster-Wingman/raw/main/noetic/launch_camera_calibration.sh
wget https://github.com/MAVProxyUser/Step2.1-GelBlaster-Wingman/raw/main/noetic/launch_moveit.sh
wget https://github.com/MAVProxyUser/Step2.1-GelBlaster-Wingman/raw/main/noetic/launch_remote_view.sh
wget https://github.com/MAVProxyUser/Step2.1-GelBlaster-Wingman/raw/main/noetic/launch_turret_control.sh
wget https://github.com/MAVProxyUser/Step2.1-GelBlaster-Wingman/raw/main/noetic/launch_turret_control_sim.sh
wget https://github.com/MAVProxyUser/Step2.1-GelBlaster-Wingman/raw/main/noetic/launch_turret_description.sh
wget https://github.com/MAVProxyUser/Step2.1-GelBlaster-Wingman/raw/main/noetic/launch_turret_object_tracker.sh
wget https://github.com/MAVProxyUser/Step2.1-GelBlaster-Wingman/raw/main/noetic/launch_turret_simple_interface.sh
wget https://github.com/MAVProxyUser/Step2.1-GelBlaster-Wingman/raw/main/noetic/noetic_turret_SDK_install.sh
wget https://github.com/MAVProxyUser/Step2.1-GelBlaster-Wingman/raw/main/noetic/turret_gazebo.sh
wget https://raw.githubusercontent.com/MAVProxyUser/Step2.1-GelBlaster-Wingman/main/noetic/launch_luxonis_yolo_publisher.sh
chmod +x *sh

sudo cp ./src/interbotix_xs_sdk/99-interbotix-udev.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "add the following entries to /etc/profile of ~/.profile :"
echo "export LD_PRELOAD=/lib/aarch64-linux-gnu/libGLdispatch.so.0:\$LD_PRELOAD"
echo "source /opt/ros/noetic/setup.sh"
echo "source /home/ubuntu/robotis_turret_ws/devel/setup.sh"

