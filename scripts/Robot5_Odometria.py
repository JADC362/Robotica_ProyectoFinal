#!/usr/bin/env python
import time
import RPi.GPIO as GPIO
import rospy
import numpy as np
from Robot5.msg import MotorVels
from Robot5.msg import GeneralPos
from geometry_msgs.msg import Pose

#Pines de los encoders en la raspberry
EncoderAD=35
EncoderBD=37
EncoderAI=11
EncoderBI=7

#Configuracion de pines de la raspberry
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO.setup(EncoderAD, GPIO.IN)
GPIO.setup(EncoderBD, GPIO.IN)
GPIO.setup(EncoderAI,GPIO.IN)
GPIO.setup(EncoderBI,GPIO.IN)

GPIO.input(EncoderAD)
GPIO.input(EncoderBD)
GPIO.input(EncoderAI)
GPIO.input(EncoderBI)

#Posiciones actual, final y de obstaculos
posicionActual = [0,0,0]
posicionFinal = [10,10,0];
obstaculos = [];

#Variable que indica el numero de pulsos por vuelta del motor con caja reductora
numeroPulsosVuelta=442

#Variables de estado de los encoders
EstadoAactualD=0
EstadoAanteriorD=GPIO.input(EncoderAD)
EstadoBactualD=0

EstadoAactualI=0
EstadoAanteriorI=GPIO.input(EncoderAI)
EstadoBactualI=0

#Tiempo general del robot
tiempoRobot = 0;

#Variable que indica si la prueba ya inicio
pruebaIniciada = False

#Grilla del mapa.
grillaMapa = []

#Tamano del mapa 2.5x2.5 (metros)
tamanoMapa = 2.5

#Tamano de grilla 0.05x0.05 (metros)
tamanoGrilla = 0.01

#Variables de representacion para publicar en los topico robot_position
pubrRobotPosition = None;

#Contador de paso de los encoders
contadorD=0
contadorI=0

#Velocidad de los motores
velocidadMD=0 #Motor derecha
velocidadMI=0 #Motor izquierdo

#Direccion de movimiento de los motores
direccionD=1
direccionI=1

#Funcion callback llamada cuando hay una actualizacion en el RobotStatus. Actualiza la informacion de estado del robot
def callbackRobotStatus(msg):
	global pruebaIniciada

	if msg.data == 1:
	 	pruebaIniciada = True
	else:
		pruebaIniciada = False

#Funcion callback llamada cuando hay una actualizacion en el GeneralPositions. Actualiza la informacion de posicion del robot y obtaculos
#Asi mismo, esta funcion da paso a la construccion de la grilla del mapa
def callbackGeneralPositions(msg):
	global obstaculos, posicionFinal

	for i in range(msg.n_obstacles):
		obstaculos.append([msg.obstacles[i].position.position.x/1000.0,msg.obstacles[i].position.position.y/1000.0,msg.obstacles[i].radius/1000.0])

	posicionFinal = [msg.goal.position.x/1000.0,msg.goal.position.y/1000.0,msg.goal.orientation.w/1000.0];
	construirGrillaMapa()

#Funcion callback llamada cuando hay una actualizacion en el robot_position. Actualiza la informacion de posicion del robot y obtaculos
def callbackRobotPosition(msg):
	global posicionActual

	posicionActual = [msg.position.x/1000.0,msg.position.y/1000.0,msg.orientation.w/1000.0];

#Funcion que determina si un obstaculo [x,y,Diametro] ocupa una grilla [x,y]
def determinarObstaculoGrilla(obstaculo,posicionGrilla):

	grillaConObstaculo = False
	distanciaObsGrilla = np.sqrt((posicionGrilla[0]-obstaculo[0])**2+(posicionGrilla[1]-obstaculo[1])**2) 

	if distanciaObsGrilla <= (razonObstaculoError*obstaculo[2]/2.0):
		grillaConObstaculo = True

	return grillaConObstaculo

#Funcion que actualiza la grilla de posicion del mapa
#Esta grilla se forma a partir de la descomposicion de celdas aproximadas
def construirGrillaMapa():
 	global grillaMapa
 	numeroCeldas = int(tamanoMapa/tamanoGrilla)
 	grillaMapa = np.zeros((numeroCeldas,numeroCeldas));

 	for i in range(numeroCeldas):
 		for j in range(numeroCeldas):
 			for k in range(len(obstaculos)):
 				y = (numeroCeldas-1-i)*tamanoGrilla
 				x = j*tamanoGrilla
 				if(determinarObstaculoGrilla(obstaculos[k],[x,y])):
 					grillaMapa[numeroCeldas-1-i][j]=1	

#Funcion encargada de convertir la respuesta de los encodores en velocidades angulares
def determinarVelocidades():
	global contadorD, contadorI, EstadoAactualD, EstadoAanteriorD, EstadoBactualD, tiempoRobot, EstadoAactualI, EstadoAanteriorI, EstadoBactualI, direccionD, direccionI, velocidadMD, velocidadMI

	EstadoAanteriorD=EstadoAactualD
	EstadoAactualD=GPIO.input(EncoderAD)
	EstadoBactualD=GPIO.input(EncoderBD)

	EstadoAanteriorI = EstadoAactualI
	EstadoAactualI = GPIO.input(EncoderAI)
	EstadoBactualI = GPIO.input(EncoderBI)

	if EstadoAactualD!=EstadoAanteriorD and EstadoAactualD==1:
		contadorD+=1
		if EstadoAactualD!=EstadoBactualD:
			direccionD=1
		else:
			direccionD=-1

	if EstadoAactualI!=EstadoAanteriorI and EstadoAactualI==1:
		contadorI+=1
		if EstadoAactualI!=EstadoBactualI:
			direccionI=1
		else:
			direccionI=-1

	if (time.time()-tiempoRobot) >= 0.01:

		velocidadMD = direccionD*(contadorD*100/numeroPulsosVuelta)*(2*np.pi)
		velocidadMI = direccionI*(contadorI*100/numeroPulsosVuelta)*(2*np.pi)

		tiempoRobot = time.time()

		print("ConD: {}, VelD:{}, ConI: {}, VelDI:{}\n".format(contadorD,velocidadMD,contadorI,velocidadMI))
		
		contadorD=0
		contadorI=0

#Funcion principal del codigo. Inicia los parametros ante ROS y mantiene este nodo en operacion, indicando que realizar
def main():

	try:

		rospy.init_node('Robot5_Odometria', anonymous=False)

		pubrRobotPosition = rospy.publisher("robot_position",Pose,queue_size=10)

		rospy.Subscriber("RobotStatus",Int32,callbackRobotStatus)
		rospy.Subscriber("GeneralPositions",GeneralPos,callbackGeneralPositions)
		rospy.Subscriber("robot_position",Pose,callbackRobotPosition)
		
		rate = rospy.Rate(10)

		while not rospy.is_shutdown():

			if pruebaIniciada:
				determinarVelocidades()

			rate.sleep()

	except Exception as e:
		raise e


#Condicion inicial para ejecutar el codigo
if __name__=='__main__':
	main()
else:
	print("Error al iniciar el codigo")