#Made by Rodrigo Garcês
#	2019

#add ROS repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#add ROS key
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

#update repository to include ROS information
sudo apt update

#install ROS full-desktop
sudo apt install -y ros-kinetic-desktop-full

#initialize rosdep
sudo rosdep init
rosdep update

#load ROS enviroment variables automatically
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

#install ROS dependencies
sudo apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools

#download V-REP
cd ~
wget http://coppeliarobotics.com/files/V-REP_PRO_EDU_V3_6_2_Ubuntu16_04.tar.xz -O vrep.tar.xz

#install vrep on current user
tar -xf vrep.tar.xz

#download project dependencies
sudo apt install -y ros-kinetic-navigation ros-kinetic-move-base ros-kinetic-gmapping

#reload bashrc file to load ROS extensions
source ~/.bashrc
