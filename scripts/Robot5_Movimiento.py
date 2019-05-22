#!/usr/bin/env python
import time
import RPi.GPIO as GPIO
import rospy
import numpy as np
from Robot5.msg import MotorVels
from std_msgs.msg import Int32
from std_msgs.msg import Float32

#Pines de Puente H en la raspberry
PWM_MotorI=18
PWM_MotorD=40
pin1LogicMotor=22 #Control llanta izquierda 
pin2LogicMotor=32 #Control llanta izquierda
pin3LogicMotor=36 #Control llanta derecha 
pin4LogicMotor=38 #Control llanta derecha

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

#Velocidad de los motores
velocidadMD=0 #Motor derecha
velocidadMI=0 #Motor izquierdo

#Variable que indica si la prueba ya inicio
pruebaIniciada = False

mayorValorEncoder = 5.6861;

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

	velocidadMD = msg.MotorD.data
	velocidadMI = msg.MotorI.data

	#print("velocidadMD:{}".format(velocidadMD))
	#print("velocidadMI:{}".format(velocidadMI))

#Funcion principal del codigo. Inicia los parametros ante ROS y mantiene este nodo en operacion, indicando que realizar
def main():

	try:
		rospy.init_node('Robot5_Movimiento', anonymous=False)

		rospy.Subscriber("RobotStatus",Int32,callbackRobotStatus)
		rospy.Subscriber("RobotMotorVels",MotorVels,callbackMotorVels)

		rate = rospy.Rate(1000)

		while not rospy.is_shutdown():

			if pruebaIniciada:
				if(velocidadMD > 0):
					GPIO.output(pin3LogicMotor, True)
					GPIO.output(pin4LogicMotor, False)
				
				else:
					GPIO.output(pin3LogicMotor, False)
					GPIO.output(pin4LogicMotor, True)
				
				if(np.abs(velocidadMD) > 6.2):
					RuedaD.ChangeDutyCycle(100)
				else:
					RuedaD.ChangeDutyCycle(np.abs(velocidadMD*100.0/6.2))
		
				
				if(velocidadMI > 0):
					GPIO.output(pin1LogicMotor, True)
					GPIO.output(pin2LogicMotor, False)
				else:
					GPIO.output(pin1LogicMotor, False)
					GPIO.output(pin2LogicMotor, True)

				if(np.abs(velocidadMI) > 6.2):
					RuedaI.ChangeDutyCycle(100)
				else:
					RuedaI.ChangeDutyCycle(np.abs(velocidadMI*100/6.2))
			
			else:
				GPIO.output(pin1LogicMotor,False)
				GPIO.output(pin2LogicMotor,False)
				GPIO.output(pin3LogicMotor,False)
				GPIO.output(pin4LogicMotor,False)
			rate.sleep()

	except Exception as e:
		raise e


#Condicion inicial para ejecutar el codigo
if __name__=='__main__':
	main()
else:
	print("Error al iniciar el codigo")
