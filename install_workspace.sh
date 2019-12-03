#Made by Rodrigo Garcês
#	2019

cd ~/target_generator
catkin build
cp src/a1slam.sh ~/a1slam.sh
cp src/a2slam.sh ~/a2slam.sh
mkdir -p ~/.rviz
cp src/default.rviz ~/.rviz/default.rviz
