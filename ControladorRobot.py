#!/usr/bin/env python
import time
import RPi.GPIO as GPIO
import rospy
import I2C_LCD_driver as LCD
import numpy as np
from geometry_msgs import Pose
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
IP="157.253.210.55";

#Funcion handle iniciada cuando se arranca el servicio start_service
def handle_start_service(msg):

	mensaje = msg;
	msg.start;
	msg.goal;
	msg.n_obstacles;
	msg.obstacles



#Funcion principal del codigo. Inicia los parametros ante ROS y mantiene este nodo en operacion, indicando que realizar
def main():

	global contador,EstadoAactual,EstadoAanterior,EstadoBactual,EstadoBanterior,tiempoinicial

	lcd.lcd_display_string(estadoInicio[0], 1)
	lcd.lcd_display_string(estadoInicio[1], 2)

	try:

		rospy.init_node('ControladorRobot5', anonymous=False)
		
		rospy.wait_for_service('ack_service')
		robotRequest = rospy.ServiceProxy('ack_service', AckService)
		AckRobot = robotRequest(group_number, IP)

		lcd.lcd_clear()
		lcd.lcd_display_string(estadoPreAck[0], 1)
		lcd.lcd_display_string(estadoPreAck[1], 2)

		while AckRobot.state == 0:
			AckRobot = robotRequest(5, IP)
			rospy.loginfo("Esperando aprobacion servidor")


		lcd.lcd_clear()
		lcd.lcd_display_string(estadoPreparado[0], 1)
		lcd.lcd_display_string(estadoPreparado[1], 2)

		start_request = rospy.Service('start_service', StartService, handle_start_service)

		rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			velocidadSg()
			pass

	except Exception as e:
		raise e
		pass


#Condicion inicial para ejecutar el codigo
if __name__=='__main__':
	main()
else:
	print("Error al iniciar el codigo")