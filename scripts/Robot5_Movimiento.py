#!/usr/bin/env python
import time
import RPi.GPIO as GPIO
import rospy
import numpy as np
from Robot5.msg import MotorVels

#Pines de Puente H en la raspberry
PWM_MotorI=18
PWM_MotorD=40
pin1LogicMotor=22 
pin2LogicMotor=32
pin3LogicMotor=36
pin4LogicMotor=38

#Pines de los encoders en la raspberry
EncoderAD=35
EncoderBD=37
EncoderAI=11
EncoderBI=7

#Configuracion de pines de la raspberry
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO.setup(pin1LogicMotor, GPIO.OUT)
GPIO.setup(pin2LogicMotor, GPIO.OUT)
GPIO.setup(pin3LogicMotor, GPIO.OUT)
GPIO.setup(pin4LogicMotor, GPIO.OUT)

GPIO.setup(PWM_MotorD, GPIO.OUT)
GPIO.setup(PWM_MotorI, GPIO.OUT)

RuedaI = GPIO.PWM(PWM_MotorI,100)
RuedaI.start(100)

RuedaD = GPIO.PWM(PWM_MotorD,100)
RuedaD.start(100)

GPIO.output(pin1LogicMotor,False)
GPIO.output(pin2LogicMotor,False)
GPIO.output(pin3LogicMotor,False)
GPIO.output(pin4LogicMotor,False)

GPIO.setup(EncoderAD, GPIO.IN)
GPIO.setup(EncoderBD, GPIO.IN)
GPIO.setup(EncoderAI,GPIO.IN)
GPIO.setup(EncoderBI,GPIO.IN)

GPIO.input(EncoderAD)
GPIO.input(EncoderBD)
GPIO.input(EncoderAI)
GPIO.input(EncoderBI)

#Variable que indica el numero de pulsos por vuelta del motor con caja reductora
numeroPulsosVuelta=442

#Velocidad de los motores
velocidadMD=0 #Motor derecha
velocidadMI=0 #Motor izquierdo

#Variable que indica si la prueba ya inicio
pruebaIniciada = False

#Funcion callback llamada cuando hay una actualizacion en el RobotStatus. Actualiza la informacion de estado del robot
def callbackRobotStatus(msg):
	global pruebaIniciada

	if msg.data == 1:
	 	pruebaIniciada = True
	else:
		pruebaIniciada = False

#Funcion callback llamada cuando hay una actualizacion en el RobotMotorVels. Actualiza la informacion de velocidad de motores del robot
def callbackMotorVels(msg):
	global velocidadMD, velocidadMI

	velocidadMD = msg.MotorD
	velocidadMI = msg.MotorI

#Funcion principal del codigo. Inicia los parametros ante ROS y mantiene este nodo en operacion, indicando que realizar
def main():

	try:
		rospy.init_node('Robot5_Movimiento', anonymous=False)

		rospy.Subscriber("RobotStatus",Int32,callbackRobotStatus)
		rospy.Subscriber("RobotMotorVels",MotorVels,callbackMotorVels)

		rate = rospy.Rate(10)

		while not rospy.is_shutdown():

			if pruebaIniciada:

				GPIO.output(pin1LogicMotor, False)
				GPIO.output(pin2LogicMotor, True)
				GPIO.output(pin3LogicMotor, False)
				GPIO.output(pin4LogicMotor, True)

			rate.sleep()

	except Exception as e:
		raise e


#Condicion inicial para ejecutar el codigo
if __name__=='__main__':
	main()
else:
	print("Error al iniciar el codigo")