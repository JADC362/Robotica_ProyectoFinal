# Robotica_ProyectoFinal

Development of a Directional Wheel Robot with Path Planning Search created in ROS (Robot Operating System). 
The system is code in Python3, running in Ubuntu 16.04 (Linux), mounted in a RaspberryPI 3. 

This is the final proyect of the course Robotics gyben in the University of the Andes (Colombia).

CONTEXTO

Este es un repositorio creado a manera de contener la solución implementada al proyecto final del programa ROBÓTICA de la Universidad de los Andes. Esta se enfoca en la creación e implementación de un robot móvil diferencial basado en ROS, que permita implementar algoritmos de filtro de Kalman, generación de grafo y mapa, calculo y movimiento de ruta optima y evación de obstaculos.

REQUERIMIENTOS DE SISTEMA

	- Ubuntu 16.04 64 bits
	- ROS Kinetic
	- Qt5 Development Libraries
	- OpenGL
	- V-REP PRO EDU

VERSION

	- Rosdistro: kinetic
	- V-rep V: 3.6
	- Ros-Pacman: 0.0.1-0
	
VERSION LIBRERIAS PYTHON

	- rospy: 1.12.14
	- pynput: 1.4
  	- numpy: 1.15.1
	- matplotlib: 2.2.3
 	- networkX: 2.2
	 
LIBRERIAS PYTHON BASE
	
	- time
	- os
	- sys
	- threading
	- random
  
INSTALACIÓN

	1) Instalar primero ROS, siguiendo el tutorial alojado en la pagina http://wiki.ros.org/kinetic/Installation/Ubuntu y crear un workspace
	2) Descargar el repositorio actual y ubicarlo en el workspace (carpeta src). 
	3) Compilar el proyecto.
	4) Iniciar todos los nodos para el movimiento del robot.
				
COMPILACIÓN

	- cd ~/catkin_ws (o dirijirse al workspace creado)
	- source devel/setup.bash
	- catkin_make
	
CREADORES

	- John Alejandro Duarte Carraco
	- Jonathan Steven Roncancio Pinzon
	- Santiago Devia Valderrama
	- Miguel Angel Mozo Reyes

