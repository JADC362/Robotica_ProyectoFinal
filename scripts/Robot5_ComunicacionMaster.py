#!/usr/bin/env python
import time
import sys
import rospy
import I2C_LCD_driver as LCD
import numpy as np
from Robot5.msg import RobotPos
from Robot5.msg import ObstacleP
from Robot5.msg import ObstaclesPos
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from master_msgs_iele3338.msg import Obstacle
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
posicionFinal = [10,10,0];
obstaculos = [];

#Variables de representacion para publicar en los topico ObstaclesPositions, RobotPositions y RobotStatus
pubObstaclesPositions = None;
pubRobotPositions = None;
pubRobotStatus = None;

#Variable que indica el estado del robot
RobotStatus = Int32();
RobotStatus.data = 0;

#Definicion de la variable lcd para el control de la pantalla lcd 16x2 por I2C
lcd = LCD.lcd()

#Funcion handle iniciada cuando se arranca el servicio start_service
def handle_start_service(msg):

	# Publica la informacion de posicion y cantidad de obstaculos en el topico RobotPositions
	RobotPositions = RobotPos();
	RobotPositions.actual = msg.start
	RobotPositions.goal = msg.goal

	pubRobotPositions.publish(RobotPositions)

	# Publica la informacion de posicion y cantidad de obstaculos en el topico ObstaclesPositions
	ObstaclesPositions = ObstaclesPos();
	ObstaclesPositions.n_obstacles = msg.n_obstacles
	ObstaclesPositions.obstacles = msg.obstacles

	pubObstaclesPositions.publish(ObstaclesPositions)

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

	global pubObstaclesPositions, pubRobotPositions, pubRobotStatus, IP

	try:
		rospy.init_node('Robot5_ComunicacionMaster', anonymous=False)

		IP = sys.argv[1];

		lcd.lcd_display_string(estadoInicio[0], 1)
		lcd.lcd_display_string(estadoInicio[1], 2)

		time.sleep(5)

		pubObstaclesPositions = rospy.Publisher('ObstaclesPositions',ObstaclesPos,queue_size=10)
		pubRobotPositions = rospy.Publisher('RobotPositions',RobotPos,queue_size=10)
		pubRobotStatus = rospy.Publisher('RobotStatus',Int32,queue_size=10)
	
		#Publica el estado cero 0 en el topico RobotStatus - Este estado representa que el robot no se le ha ordenado iniciar
		pubRobotStatus.publish(RobotStatus); 

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

		rospy.Subscriber("RobotStatus",Int32,callbackRobotStatus)

		rate = rospy.Rate(10)

		while not rospy.is_shutdown():

			if RobotStatus == 2:
				numero=np.random.randint(1000,9999)
				terminarServicio(numero);

			rate.sleep()

	except Exception as e:
		raise e


#Condicion inicial para ejecutar el codigo
if __name__=='__main__':
	main()
else:
	print("Error al iniciar el codigo")