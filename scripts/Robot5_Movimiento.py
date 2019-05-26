#!/usr/bin/env python
import time as tm
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
RuedaI.start(0)

RuedaD = GPIO.PWM(PWM_MotorD,100)
RuedaD.start(0)

GPIO.output(pin1LogicMotor,False)
GPIO.output(pin2LogicMotor,False)
GPIO.output(pin3LogicMotor,False)
GPIO.output(pin4LogicMotor,False)

#Velocidad de los motores
velocidadMDCine=0 #Motor derecha - Cinematica
velocidadMICine=0 #Motor izquierdo - Cinematica
velocidadMDOdo=0 #Motor derecha - Odometria
velocidadMIOdo=0 #Motor izquierdo - Odometria

#Variable que indica si la prueba ya inicio
pruebaIniciada = False

#Tiempo inicial de ejecucion del nodo
tiempoInicial = 0;

#Constantes de control Kp, Ki
k = np.array([1, 0.0]);

#Vector de control del sistema pwm
pwmControl = np.array([0,0]);

velocidadMaxima = 6.2;

vectorIntegral1 = np.zeros([1,10]);
vectorIntegral2 = np.zeros([1,10]);

#Funcion callback llamada cuando hay una actualizacion en el RobotStatus. Actualiza la informacion de estado del robot
def callbackRobotStatus(msg):
	global pruebaIniciada

	if msg.data == 1:
	 	pruebaIniciada = True
	else:
		pruebaIniciada = False

#Funcion callback llamada cuando hay una actualizacion en el RobotMotorVels. Actualiza la informacion de velocidad de motores del robot
def callbackMotorVelsCine(msg):
	global velocidadMDCine, velocidadMICine

	velocidadMDCine = msg.MotorD.data
	velocidadMICine = msg.MotorI.data

#Funcion callback llamada cuando hay una actualizacion en el RobotMotorVels. Actualiza la informacion de velocidad de motores del robot
def callbackMotorVelsOdo(msg):
	global velocidadMDOdo, velocidadMIOdo

	velocidadMDOdo = msg.MotorD.data
	velocidadMIOdo = msg.MotorI.data

def controlPWM():
	global tiempoInicial, vectorIntegral1,vectorIntegral2, pwmControl
	tiempoActual = tm.time();

	errorControl = np.array([velocidadMDCine,velocidadMICine])-np.array([velocidadMDOdo,velocidadMIOdo]);
	deltaT = tiempoActual-tiempoInicial;
			
	vectorIntegral1[1:] = vectorIntegral1[0:-1];
	vectorIntegral1[0] = deltaT*errorControl[0];

	vectorIntegral2[1:] = vectorIntegral2[0:-1];
	vectorIntegral2[0] = deltaT*errorControl[1];
	
	integralControl = np.array([np.sum(vectorIntegral1),np.sum(vectorIntegral2)]);
				
	tiempoInicial=tiempoActual;	
				
	pwmControl = np.array([velocidadMDOdo,velocidadMIOdo]) + (k[0]*errorControl+k[1]*integralControl);

	pwmControl=(pwmControl*100)/velocidadMaxima;	

	#print(pwmControl);

	if np.abs(pwmControl[0]) > 100:
		pwmControl[0] = ((pwmControl[0])/np.abs(pwmControl[0]))*100;

	if np.abs(pwmControl[1]) > 100:
		pwmControl[1] = ((pwmControl[1])/np.abs(pwmControl[1]))*100;
	

#Funcion principal del codigo. Inicia los parametros ante ROS y mantiene este nodo en operacion, indicando que realizar
def main():

	global tiempoInicial

	try:
		rospy.init_node('Robot5_Movimiento', anonymous=False)

		rospy.Subscriber("RobotStatus",Int32,callbackRobotStatus)
		rospy.Subscriber("RobotMotorVelsCine",MotorVels,callbackMotorVelsCine)
		rospy.Subscriber("RobotMotorVelsOdo",MotorVels,callbackMotorVelsOdo)

		rate = rospy.Rate(10000)

		tiempoInicial = tm.time()

		while not rospy.is_shutdown():

			if pruebaIniciada:
				controlPWM();
				if(pwmControl[0] > 0):
					GPIO.output(pin3LogicMotor, False)
					GPIO.output(pin4LogicMotor, True)
				
				else:
					GPIO.output(pin3LogicMotor, True)
					GPIO.output(pin4LogicMotor, False)
				
				RuedaD.ChangeDutyCycle(np.abs(pwmControl[0]))
		
				
				if(pwmControl[1] > 0):
					GPIO.output(pin1LogicMotor, False)
					GPIO.output(pin2LogicMotor, True)
				else:
					GPIO.output(pin1LogicMotor, True)
					GPIO.output(pin2LogicMotor, False)

				RuedaI.ChangeDutyCycle(np.abs(pwmControl[1]))
			
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
