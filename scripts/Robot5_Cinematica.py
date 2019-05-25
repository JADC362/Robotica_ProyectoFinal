#!/usr/bin/env python
import time
import rospy
import numpy as np
import matplotlib.pyplot as plt
from Robot5.msg import MotorVels
from Robot5.msg import GeneralPos
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
import networkx as nx

#Posiciones actual, final y deseada
posicionActual = [0,0,0]
posicionFinal = [1,1,0]*1000;
posicionDeseada = [0,0,0];

#Matriz de obstaculos, cada fila corresponde a un obstaculo, y cada columna a un dato
#Columna 1: PosX, Columna 2: PosY, Columna 3: Diametros
obstaculos = [];

#Tiempo general del robot
tiempoRobot = 0;

#Constante del sistema de control
k = [0.04*10,0.3*0,0.02*0]

#Variable de umbral de error superado cuando el robot se acerca a una posicion
umbralSuperado = False;

#Variable de umbral de error superado cuando el robot se acerca a una posicion
errorRho = 10;

#Vector que contiene los parametros de las ruedas del robot: alpha, beta, r, l
paraRuedaI = [np.pi/2.0,np.pi,35.0,80.0]
paraRuedaD = [-np.pi/2.0,0,35.0,80.0]

#Variable que indica si la prueba ya inicio
pruebaIniciada = False

#Grilla del mapa.
grillaMapa = []

#Tamano del mapa 2500x2500 (milimetros)
tamanoMapa = 2500

#Tamano de grilla 5x5 (milimetros)
tamanoGrilla = 50

#Razon de aumento de los radios de los obstaculos
razonObstaculoError = 1.5

#Grafo
G = None

#Ruta optima en el grafo al punto final
ruta = None

#Variable que indica a que posicion de la ruta optima debe dirigirse el robot actualmente
contadorPasos=0

#Variables que indican la velocidad estimada del motor derecho 
velocidadMD = 0;
velocidadMI = 0;

#Variables de representacion para publicar en el topico RobotMotorVels
pubRobotMotorVels = None;

#Variable que indica la velocidad de los motores del robot
RobotMotorVels = MotorVels();
RobotMotorVels.MotorD = 0.0;
RobotMotorVels.MotorI = 0.0;

#Variable que indica si ya hubo actualizacion de la posicion del robot en el topico robot_position
callPos = False

#Variable de error que representa el error total de posicion 
errorMax=(tamanoGrilla)

#Variable que representa si ya se sobrepaso el umbral de error
umbralSuperado = False

#Variable que representa el umbral de error aceptado en theta
errorMaxTheta = 0.02

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
	global posicionFinal, obstaculos

	for i in range(msg.n_obstacles):
		obstaculos.append([msg.obstacles[i].position.position.x,msg.obstacles[i].position.position.y,msg.obstacles[i].radius])
	
	posicionFinal = [msg.goal.position.x,msg.goal.position.y,msg.goal.orientation.w];
	
	construirGrillaMapa()

#Funcion callback llamada cuando hay una actualizacion en el robot_position. Actualiza la informacion de posicion del robot y obtaculos
def callbackRobotPosition(msg):
	global posicionActual, callPos

	posicionActual = [msg.position.x,msg.position.y,msg.orientation.w];

	callPos = True;


#Funcion que determina si un obstaculo [x,y,Diametro] ocupa una grilla [x,y]
def determinarObstaculoGrilla(obstaculo,posicionGrilla):

	grillaConObstaculo = False
	distanciaObsGrilla = np.sqrt((posicionGrilla[0]-obstaculo[0])**2+(posicionGrilla[1]-obstaculo[1])**2) 

	if distanciaObsGrilla <= (razonObstaculoError*obstaculo[2]/2.0):
		grillaConObstaculo = True

	return grillaConObstaculo

#Funcion que actualiza la grilla de posicion del mapa
#Esta grilla se forma a partir de la descomposicion de celdas aproximadas
#Asi mismo, esta funcion da paso a la construccion del grafo del mapa
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

 	#plt.figure()
 	#plt.imshow(grillaMapa,origin='lower')	
 	#plt.show()

 	construirGrafo();

#Funcion que contruye el grafo a partir de la grilla de ocupacion
#Los nodos corresponden al centro de una grilla, y los arcos a las conexines entre ellas
#Asi mismo, esta funcion da paso al calculo de la ruta optima
def construirGrafo():	
	global G	
	pesoObstaculo=1e9
	G = nx.Graph()
 	numeroCeldas = int(tamanoMapa/tamanoGrilla)

	pos = {}
	for i in range(numeroCeldas):
		for j in range(numeroCeldas):
			pos['{},{}'.format(i,j)]= (i,j)

	vector = []
	G.add_nodes_from(pos)

	for m in range(numeroCeldas):
		i=numeroCeldas-1-m
		for n in range(numeroCeldas):
			j=numeroCeldas-1-n
			if (i > 0 and j > 0) and (i < numeroCeldas-1 and j < numeroCeldas-1):

				pesoI = tamanoGrilla
				if grillaMapa[i+1,j]==1:
					pesoI = pesoObstaculo
					
				pesoA = tamanoGrilla
				if grillaMapa[i-1,j]==1:
					pesoA = pesoObstaculo
					
				pesoDe = tamanoGrilla
				if grillaMapa[i,j+1]==1:
					pesoDe = pesoObstaculo
					
				pesoIz = tamanoGrilla
				if grillaMapa[i,j-1]==1:
					pesoIz = pesoObstaculo
					

				pesoEsqID = np.sqrt(2)*tamanoGrilla					
				if grillaMapa[i+1,j+1]==1:
					pesoEsqID = pesoObstaculo

				pesoEsqSI = np.sqrt(2)*tamanoGrilla
				if grillaMapa[i-1,j-1]==1:
					pesoEsqSI = pesoObstaculo
					
				pesoEsqSD = np.sqrt(2)*tamanoGrilla
				if grillaMapa[i-1,j+1]==1:
					pesoEsqSD = pesoObstaculo

				pesoEsqII = np.sqrt(2)*tamanoGrilla
				if grillaMapa[i+1,j-1]==1:
					pesoEsqII = pesoObstaculo

				G.add_edge("{},{}".format(i,j),"{},{}".format(i+1,j),weight=pesoI)
				G.add_edge("{},{}".format(i,j),"{},{}".format(i-1,j),weight=pesoA)
				G.add_edge("{},{}".format(i,j),"{},{}".format(i,j+1),weight=pesoDe)
				G.add_edge("{},{}".format(i,j),"{},{}".format(i,j-1),weight=pesoIz)

				G.add_edge("{},{}".format(i,j),"{},{}".format(i+1,j+1),weight=pesoEsqID)
				G.add_edge("{},{}".format(i,j),"{},{}".format(i-1,j-1),weight=pesoEsqSI)
				G.add_edge("{},{}".format(i,j),"{},{}".format(i-1,j+1),weight=pesoEsqSD)
				G.add_edge("{},{}".format(i,j),"{},{}".format(i+1,j-1),weight=pesoEsqII)

	pathPlaning();

#Funcion que detecta si un punto esta en un obstaculo 
# y en caso de que este aproxime a un punto libre(sin obstaculo) mas cercano
# retorna la coordenada x,y del punto en el grafo 
def buscarPosicionMasCercana(x,y):
	xc=0
	yc=0

	distanciaMasCercana = 1e9
	distancia = 0

	for i in range(len(grillaMapa)):
		for j in range(len(grillaMapa)):
			if grillaMapa[i][j]==0:
				distancia =np.sqrt((i-x)**2+(j-y)**2)
				if distancia < distanciaMasCercana:
					distanciaMasCercana=distancia
					xc=i
					yc=j
	return (xc,yc)

#Funcion que calcula la heuristica entre un nodo de entrada y uno de salida utilizando como medida la distancia Manhattan
def heuristica(nodeI,targetI):
	D = 3

	[x1,y1]=nodeI.split(",")
	x1=int(x1)
	y1=int(y1)

	[x2,y2]=targetI.split(",")
	x2=int(x2)
	y2=int(y2)

	DManhattan=D*(np.absolute(x1-x2)+np.absolute(y1-y2))

	return DManhattan

#Funcion que calcula la ruta optima al punto final haciendo uso del algoritmo A*
def pathPlaning():

	global ruta, contadorPasos
	contadorPasos=0

	xf=int(posicionFinal[0]/tamanoGrilla)
	yf=int(posicionFinal[1]/tamanoGrilla)

	if(grillaMapa[xf][yf]==1):
		[xf,yf]=buscarPosicionMasCercana(xf,yf) 

	xi=int(posicionActual[0]/tamanoGrilla)
	yi=int(posicionActual[1]/tamanoGrilla)

	nodoInicial = "{},{}".format(xi,yi)
	nodoFinal = "{},{}".format(xf,yf)

	ruta = nx.astar_path(G,nodoInicial,nodoFinal,heuristica)
	
	print("Imprimir Ruta ")
	print(ruta)
	print("\n")
	
	CalcularPosicionDeseada()

#Funcion que calcula la posicion x,y,theta a donde se debe mover el robot
def CalcularPosicionDeseada():
	global contadorPasos, posicionDeseada

	posicionDeseada=[0,0,0]
	[i,j]=ruta[contadorPasos].split(",")

	x=float(int(i)*tamanoGrilla)
	y=float(int(j)*tamanoGrilla)

	posicionDeseada[0]=x
	posicionDeseada[1]=y

	if contadorPasos==(len(ruta)-1):
		posicionDeseada[2]=posicionFinal[2]
	else:
		posicionDeseada[2]=posicionActual[2]

	if contadorPasos < (len(ruta)-1):
		contadorPasos+=1

#Obtiene la posicion del robot en coordenadas polares rho, alpha, beta
def obtenerPosicionPol(puntoFinal):
	global umbralSuperado, errorRho

	rho = np.sqrt((puntoFinal[0]-posicionActual[0])**2+(puntoFinal[1]-posicionActual[1])**2)

	if rho <= errorRho:
		umbralSuperado = True

	if not umbralSuperado:
		alpha = -posicionActual[2]+np.arctan2((puntoFinal[1]-posicionActual[1]),(puntoFinal[0]-posicionActual[0]))
	else:
		alpha = 0;
		rho = 0;

	errorTheta = posicionActual[2]-puntoFinal[2]

	if umbralSuperado and np.absolute(errorTheta)<=errorMaxTheta:
		beta = 0
	else:
		beta = -alpha-posicionActual[2]

	return np.asarray([rho,alpha,beta])

#Funcion encargada de determinar las velocidades de los motores en cinametica inversa a partir de una ley de control
def calcularCinematicaRobot(puntoFinal):
	global velocidadMD, velocidadMI, paraRuedaD, paraRuedaI, RobotMotorVels, pubRobotMotorVels
	
	#Obtencion del error de posicion en coordenadas polares
	posPol = np.asarray([0,0,puntoFinal[2]])-obtenerPosicionPol(puntoFinal)
	
	#print("Error:{}".format(posPol));
	#Ley de control aplicada para encontrar v y w
	vecVelLinearAngular = np.asarray([k[0]*posPol[0],k[1]*posPol[1]+k[2]*posPol[2]])
	
	#A partir de v y w se determina el vector [x',y',theta']
	veloCar = [(vecVelLinearAngular[0])*(np.cos(posicionActual[2])),(vecVelLinearAngular[0])*(np.sin(posicionActual[2])),vecVelLinearAngular[1]]

	#Matriz de transformacion del marco global al local
	matrixR = [[np.cos(posicionActual[2]),np.sin(posicionActual[2]),0],[-np.sin(posicionActual[2]),np.cos(posicionActual[2]),0],[0,0,1]]
	
	#Obtencion de las velocidades aplicadas a cada motor. Cinematica aplicada
	velocidadMD = np.dot([np.sin(paraRuedaD[0]+paraRuedaD[1]),-np.cos(paraRuedaD[0]+paraRuedaD[1]),-(paraRuedaD[3])*(np.cos(paraRuedaD[1]))],np.dot(matrixR,veloCar))/paraRuedaD[2]
	velocidadMI = np.dot([np.sin(paraRuedaI[0]+paraRuedaI[1]),-np.cos(paraRuedaI[0]+paraRuedaI[1]),-(paraRuedaI[3])*(np.cos(paraRuedaI[1]))],np.dot(matrixR,veloCar))/paraRuedaI[2]

	RobotMotorVels = MotorVels();
	RobotMotorVels.MotorD = Float32(velocidadMD);
	RobotMotorVels.MotorI = Float32(velocidadMI);

	#print(velocidadMD)
	#print(velocidadMI)

	pubRobotMotorVels.publish(RobotMotorVels);

#Funcion principal del codigo. Inicia los parametros ante ROS y mantiene este nodo en operacion, indicando que realizar
def main():
	try:
		global pubRobotMotorVels, callPos
		rospy.init_node('Robot5_Cinematica', anonymous=False)

		pubRobotMotorVels=rospy.Publisher("RobotMotorVelsCine",MotorVels,queue_size=10)

		rospy.Subscriber("RobotStatus",Int32,callbackRobotStatus)
		rospy.Subscriber("GeneralPositions",GeneralPos,callbackGeneralPositions)
		rospy.Subscriber("robot_position",Pose,callbackRobotPosition)

		RobotMotorVels = MotorVels();
		RobotMotorVels.MotorD = Float32(0);
		RobotMotorVels.MotorI = Float32(0);
		pubRobotMotorVels.publish(RobotMotorVels);

		rate = rospy.Rate(1000)

		while not rospy.is_shutdown():

			if pruebaIniciada and ruta != None and callPos:

				errorPosicionDeseada = np.sqrt((posicionDeseada[0]-posicionActual[0])**2+(posicionDeseada[1]-posicionActual[1])**2)
				#print("Error Actual: {}".format(errorPosicionDeseada));
				if errorPosicionDeseada <= errorMax:
					CalcularPosicionDeseada();
					#print("Posicion Deseada: ")
					#print(posicionDeseada)
					#print("\n")
					
				calcularCinematicaRobot(posicionDeseada)
				callPos = False;

			rate.sleep()

	except Exception as e:
		raise e


#Condicion inicial para ejecutar el codigo
if __name__=='__main__':
	main()
else:
	print("Error al iniciar el codigo")
