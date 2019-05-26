#!/usr/bin/env python
import time
import sys
import rospy
import I2C_LCD_driver as LCD
import numpy as np
from Robot5.msg import GeneralPos
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from master_msgs_iele3338.srv import AckService
from master_msgs_iele3338.srv import StartService, StartServiceResponse
from master_msgs_iele3338.srv import EndService

#Constantes que indican los estados de la raspberry
estadoInicio = ["Hola mundo","R. Olivaw V 1.0"];
estadoPreAck = ["Preparado","Enviando ACK"];
estadoPreparado = ["Listo","Arranque"];
estadoMovimiento = ["Moviendose","Runnnn!!"];
estadoParar = ["Detenido","Completado"];

#Parametros necesarios para la conexion con el master
group_number = 5;
IP="0.0.0.0";

#Posiciones actual, final y de obstaculos
posicionInicial = [0,0,0]
posicionFinal = [1,1,0]*1000;
obstaculos = [];

#Variables de representacion para publicar en los topico GeneralPositions, RobotPosition y RobotStatus
pubGeneralPositions = None;
pubRobotPosition = None;
pubRobotStatus = None;

#Variable que indica el estado del robot
RobotStatus = Int32();
RobotStatus.data = 0;

#Definicion de la variable lcd para el control de la pantalla lcd 16x2 por I2C
lcd = LCD.lcd()

#Funcion handle iniciada cuando se arranca el servicio start_service
def handle_start_service(msg):

	# Publica la informacion de posicion y cantidad de obstaculos en el topico RobotPosition
	RobotPosition = Pose();
	RobotPosition.position = msg.start.position
	RobotPosition.orientation = msg.start.orientation

	pubRobotPosition.publish(RobotPosition)

	# Publica la informacion de posicion y cantidad de obstaculos en el topico GeneralPositions
	GeneralPositions = GeneralPos();
	GeneralPositions.start = msg.start
	GeneralPositions.goal = msg.goal
	GeneralPositions.n_obstacles = msg.n_obstacles
	GeneralPositions.obstacles = msg.obstacles

	pubGeneralPositions.publish(GeneralPositions)

	#Publica el estado cero 1 en el topico RobotStatus - Este estado representa que el robot se le ha ordenado iniciar
	RobotStatus.data = 1;

	pubRobotStatus.publish(RobotStatus); 

	return StartServiceResponse()

#Funcion encargada de enviar la peticion de servicio para finalizar la prueba una vez se alcanzo el objetivo
def terminarServicio(msg):
	end_Request = rospy.ServiceProxy('end_service', EndService)
	EndRobot = end_Request(msg)
	rospy.loginfo(EndRobot)

#Funcion callback que se llama cuando el topic RobotStatus tiene una actualizacion
def callbackRobotStatus(msg):
	RobotStatus.data = msg.data;

#Funcion principal del codigo. Inicia los parametros ante ROS y mantiene este nodo en operacion, indicando que realizar
def main():

	global pubGeneralPositions, pubRobotPosition, pubRobotStatus, IP

	try:
		rospy.init_node('Robot5_ComunicacionMaster', anonymous=False)

		IP = sys.argv[1];

		lcd.lcd_display_string(estadoInicio[0], 1)
		lcd.lcd_display_string(estadoInicio[1], 2)

		time.sleep(5)

		pubGeneralPositions = rospy.Publisher('GeneralPositions',GeneralPos,queue_size=10)
		pubRobotPosition = rospy.Publisher('robot_position',Pose,queue_size=10)
		pubRobotStatus = rospy.Publisher('RobotStatus',Int32,queue_size=10)
	
		#Publica el estado cero 0 en el topico RobotStatus - Este estado representa que el robot no se le ha ordenado iniciar
		pubRobotStatus.publish(RobotStatus); 

		rospy.Subscriber("RobotStatus",Int32,callbackRobotStatus)

		lcd.lcd_clear()
		lcd.lcd_display_string(estadoPreAck[0], 1)
		lcd.lcd_display_string(estadoPreAck[1], 2)
		
		rospy.wait_for_service('ack_service')
		robotRequest = rospy.ServiceProxy('ack_service', AckService)
		AckRobot = robotRequest(group_number, IP)
		
		while AckRobot.state == 0:
			AckRobot = robotRequest(5, IP)
			rospy.loginfo("Esperando servidor AckService")

		lcd.lcd_clear()
		lcd.lcd_display_string(estadoPreparado[0], 1)
		lcd.lcd_display_string(estadoPreparado[1], 2)

		start_request = rospy.Service('start_service', StartService, handle_start_service)


		rate = rospy.Rate(1000)

		while not rospy.is_shutdown():

			if RobotStatus == 2:
				numero = 1234;
				terminarServicio(numero);

			rate.sleep()

	except Exception as e:
		raise e


#Condicion inicial para ejecutar el codigo
if __name__=='__main__':
	main()
else:
	print("Error al iniciar el codigo")
