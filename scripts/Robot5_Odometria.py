#!/usr/bin/env python
import time
import RPi.GPIO as GPIO
import rospy
import numpy as np
from Robot5.msg import GeneralPos
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from Robot5.msg import MotorVels
from master_msgs_iele3338.msg import Covariance

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
posicionFinal = [1,1,0]*1000;
obstaculos = [];

#Matriz de error de la posicion actual
matrizCovarianza = []

#Variable que indica el numero de pulsos por vuelta del motor con caja reductora
numeroPulsosVuelta=442.0

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

#Tamano del mapa 2500x2500 (milimetros)
tamanoMapa = 2500

#Tamano de grilla 0.05x0.05 (milimetros)
tamanoGrilla = 50

#Variables de representacion para publicar en los topico robot_position, robot_uncertainty y RobotMotorVelsOdo
pubRobotPosition = None;
pubRobotCovariance = None;
pubRobotMotorVelsOdo = None;

#Contador de paso de los encoders
contadorD=0
contadorI=0

#Velocidad lineal de los motores en m/s
velocidadMD=0 #Motor derecha
velocidadMI=0 #Motor izquierdo

#Direccion de movimiento de los motores
direccionD=0
direccionI=0

#Vector que contiene los parametros de las ruedas del robot: alpha, beta, r, l
paraRuedaI = [np.pi/2.0,0,35.0,80.0]
paraRuedaD = [-np.pi/2.0,-np.pi,35.0,80.0]

#Proporcionalidad de errores generados por las ruedas en su moviminento
errorKD = 0.1;
errorKI = 0.1;

#Razon de aumento de los radios de los obstaculos
razonObstaculoError = 1.0

#Tiempo durante el cual se estima la velocidad
tiempoMedicionVelocidad = 0.01

#Funcion callback llamada cuando hay una actualizacion en el RobotMotorVels. Actualiza la informacion de velocidad de motores del robot
def callbackMotorVels(msg):
	global direccionD, direccionI

	velocidadMD = msg.MotorD.data
	velocidadMI = msg.MotorI.data

	if velocidadMD > 0:
		direccionD = 1;
	else:
		direccionD = -1;
	if velocidadMI > 0:
		direccionI = 1;
	else:
		direccionI = -1;

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
		obstaculos.append([msg.obstacles[i].position.position.x,msg.obstacles[i].position.position.y,msg.obstacles[i].radius])

	posicionFinal = [msg.goal.position.x,msg.goal.position.y,msg.goal.orientation.w];
	construirGrillaMapa()

#Funcion callback llamada cuando hay una actualizacion en el robot_position. Actualiza la informacion de posicion del robot y obtaculos
def callbackRobotPosition(msg):
	global posicionActual
	
	if (posicionActual[0]==0) and (posicionActual[1]==0) and (posicionActual[2]==0):
		posicionActual = [msg.position.x,msg.position.y,msg.orientation.w];

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
#Asi mismo, la funcion se encarga de llamar la funcion para actualizar la posicion que cree el robot
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
				

	if EstadoAactualI!=EstadoAanteriorI and EstadoAactualI==1:
		contadorI+=1

	if (time.time()-tiempoRobot) >= tiempoMedicionVelocidad:


		#print(contadorD)
		#print(contadorI)

		velocidadMD = direccionD*(contadorD*100/numeroPulsosVuelta)*(2*np.pi)
		velocidadMI = direccionI*(contadorI*100/numeroPulsosVuelta)*(2*np.pi)

		tiempoRobot = time.time()

		#print("ConD: {}, VelD:{}, ConI: {}, VelDI:{}\n".format(contadorD,velocidadMD,contadorI,velocidadMI))
		
		contadorD=0
		contadorI=0

		RobotMotorVels = MotorVels();
		RobotMotorVels.MotorD = Float32(velocidadMD);
		RobotMotorVels.MotorI = Float32(velocidadMI);

		pubRobotMotorVelsOdo.publish(RobotMotorVels);
		actualizarPosicionActual()

#Funcion encargada de actualizar la posicion actual que cree el robot
#Asi mismo, la funcion se encarga de llamar al metodo para actualizar la matriz de covarianza
#Por ultimo, la funcion publica esta posicion nueva en el topico robot_position
def actualizarPosicionActual():
	global posicionActual

	DeltaSr = velocidadMD*tiempoMedicionVelocidad*paraRuedaD[2];
	DeltaSi = velocidadMI*tiempoMedicionVelocidad*paraRuedaI[2];

	DeltaS = (DeltaSr+DeltaSi)/2.0;
	DeltaT = (DeltaSr-DeltaSi)/(2.0*paraRuedaD[3]);

	actualizarErrorPropagado(DeltaS,DeltaT) #Se tiene que calcular primero el error antes de calcular la nueva posicion

	adicionPosicion = [DeltaS*np.cos(posicionActual[2]+DeltaT/2.0),DeltaS*np.sin(posicionActual[2]+DeltaT/2.0),DeltaT]
	posicionActual = np.array(posicionActual) + np.array(adicionPosicion)

	posicionRobot = Pose();
	posicionRobot.position.x = posicionActual[0]
	posicionRobot.position.y = posicionActual[1]
	posicionRobot.position.z = 0

	posicionRobot.orientation.x = 0
	posicionRobot.orientation.y = 0
	posicionRobot.orientation.z = 0
	posicionRobot.orientation.w = posicionActual[2]

	pubRobotPosition.publish(posicionRobot)

#Funcion encargada de actualizar la matriz de covarianza de error en el movimiento del robot
def actualizarErrorPropagado(DeltaS,DeltaT):
	global matrizCovarianza, pubRobotCovariance

	DeltaSr = velocidadMD*tiempoMedicionVelocidad*paraRuedaD[2];
	DeltaSi = velocidadMI*tiempoMedicionVelocidad*paraRuedaI[2];

	DeltaS = (DeltaSr+DeltaSi)/2.0;
	DeltaT = (DeltaSr-DeltaSi)/(2.0*paraRuedaD[3]);

	gradientePosicion = np.array([[1,0,-DeltaS*np.sin(posicionActual[2]+DeltaT/2.0)],[0,1,DeltaS*np.cos(posicionActual[2]+DeltaT/2.0)],[0,0,1]]);
	
	gradienteRuedasFila1 = np.array([((1/2.0)*np.cos(posicionActual[2]+DeltaT/2.0))-((1/2.0)*(DeltaS/(2.0*paraRuedaD[3]))*np.sin(posicionActual[2]+DeltaT/2.0)),((1/2.0)*np.cos(posicionActual[2]+DeltaT/2.0))+((1/2.0)*(DeltaS/(2.0*paraRuedaD[3]))*np.sin(posicionActual[2]+DeltaT/2.0))])
	gradienteRuedasFila2 = np.array([((1/2.0)*np.sin(posicionActual[2]+DeltaT/2.0))+((1/2.0)*(DeltaS/(2.0*paraRuedaD[3]))*np.cos(posicionActual[2]+DeltaT/2.0)),((1/2.0)*np.sin(posicionActual[2]+DeltaT/2.0))-((1/2.0)*(DeltaS/(2.0*paraRuedaD[3]))*np.cos(posicionActual[2]+DeltaT/2.0))])
	gradienteRuedas = np.array([gradienteRuedasFila1,gradienteRuedasFila2,[(1/(2.0*paraRuedaD[3])),-(1/(2.0*paraRuedaD[3]))]])

	matrizCovarianzaRuedas = np.array([[errorKD*np.abs(DeltaSr),0],[0,errorKI*np.abs(DeltaSi)]])

	matrizCovarianza = np.matmul(np.matmul(gradientePosicion,matrizCovarianza),np.transpose(gradientePosicion))+np.matmul(np.matmul(gradienteRuedas,matrizCovarianzaRuedas),np.transpose(gradienteRuedas))

	matrizErrorCovarianza = Covariance()
	matrizErrorCovarianza.sigma11 = matrizCovarianza[0][0]
	matrizErrorCovarianza.sigma12 = matrizCovarianza[0][1]
	matrizErrorCovarianza.sigma13 = matrizCovarianza[0][2]
	matrizErrorCovarianza.sigma21 = matrizCovarianza[1][0]
	matrizErrorCovarianza.sigma22 = matrizCovarianza[1][1]
	matrizErrorCovarianza.sigma23 = matrizCovarianza[1][2]
	matrizErrorCovarianza.sigma31 = matrizCovarianza[2][0]
	matrizErrorCovarianza.sigma32 = matrizCovarianza[2][1]
	matrizErrorCovarianza.sigma33 = matrizCovarianza[2][2]
	
	pubRobotCovariance.publish(matrizErrorCovarianza);

#Funcion principal del codigo. Inicia los parametros ante ROS y mantiene este nodo en operacion, indicando que realizar
def main():

	global pubRobotPosition, pubRobotCovariance, matrizCovarianza, pubRobotMotorVelsOdo

	try:

		rospy.init_node('Robot5_Odometria', anonymous=False)

		pubRobotPosition = rospy.Publisher("robot_position",Pose,queue_size=10)
		pubRobotCovariance = rospy.Publisher("robot_uncertainty",Covariance,queue_size=10)
		pubRobotMotorVelsOdo = rospy.Publisher("RobotMotorVelsOdo",MotorVels,queue_size=10)
		
		rospy.Subscriber("RobotStatus",Int32,callbackRobotStatus)
		rospy.Subscriber("GeneralPositions",GeneralPos,callbackGeneralPositions)
		rospy.Subscriber("robot_position",Pose,callbackRobotPosition)
		rospy.Subscriber("RobotMotorVelsCine",MotorVels,callbackMotorVels)

		matrizCovarianza = np.zeros([3,3])
		
		rate = rospy.Rate(1000)

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
