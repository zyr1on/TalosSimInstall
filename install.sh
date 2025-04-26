#!/bin/bash

dir="./src/cart_sim/models/"

RED='\033[1;31m'  
GREEN='\033[1;32m' 
YELLOW='\033[1;33m'
NC='\033[0m'

sudo echo -e "\n${GREEN}[+] Installing Dependencies. Please wait.${NC}"  
chmod +x *
sudo apt-get install gcc g++ git curl wget libqt5x11extras5-dev ros-noetic-fake-localization ros-noetic-pcl-ros ros-noetic-dynamic-reconfigure ros-noetic-roslint  ros-noetic-pcl-conversions ros-noetic-gazebo-ros-pkgs ros-noetic-pcl-conversions ros-noetic-roscpp ros-noetic-rospy ros-noetic-std-msgs ros-noetic-geometry-msgs  ros-noetic-joy  ros-noetic-message-generation  -y >/dev/null
echo -e "${GREEN}[+] Installing Gazebo9.Please wait.${NC}"  
sudo apt-get install gazebo9 libgazebo9-dev  -y >/dev/null
echo -e "${YELLOW}[?] Looking for models.${NC}"  
if [ ! -d "$dir" ]; then
	echo -e "${YELLOW}[?] Models File Not found. Installing... Please wait${NC}"  
	wget https://filebin.net/dlydd67y3b3ydhqz/models-20250426T091428Z-001.zip >/dev/null
	echo -e "${YELLOW}[?] Unzipping model file... Please wait${NC}"  
	unzip models-20250426T091428Z-001.zip >/dev/null
	mv models/ src/cart_sim/
	rm models-20250426T091428Z-001.zip
else
	echo -e "${GREEN}[+] Models File found.${NC}" 
fi
echo -e "${YELLOW}[?] Compiling...Please wait${NC}"
sleep 3
catkin_make
source devel/setup.bash
echo -e "${GREEN}[+] SIM INSTALLED you can run via './run.sh' file.${NC}" 
