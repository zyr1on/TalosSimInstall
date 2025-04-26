#!/bin/bash

source devel/setup.bash
ps aux | grep '[g]azebo' | awk '{print $2}' | xargs kill
ps aux | grep '[r]vizz' | awk '{print $2}' | xargs kill
roslaunch cart_sim cart_sim.launch
