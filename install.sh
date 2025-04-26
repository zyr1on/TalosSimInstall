chmod +x *
sudo apt-get install ros-noetic-pcl-ros ros-noetic-gazebo-ros-pkgs ros-noetic-pcl-conversions gazebo9 libgazebo9-dev git curl
wget https://filebin.net/dlydd67y3b3ydhqz/models-20250426T091428Z-001.zip
unzip models-20250426T091428Z-001.zip
mv models/ src/cart_sim/
rm models-20250426T091428Z-001.zip
catkin_make
source devel/setup.bash
