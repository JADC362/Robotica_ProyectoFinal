#!/usr/bin/env bash

cd ~/catkin_ws/src/Robot5/scripts/
chmod +x *.py
cd ~/catkin_ws/
source devel/setup.bash 
rosrun Robot5 Robot5_ComunicacionMaster.py $(hostname -I) &
rosrun Robot5 Robot5_Cinematica.py &
rosrun Robot5 Robot5_Movimiento.py &