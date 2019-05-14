#!/usr/bin/env python
import time
import RPi.GPIO as GPIO
import rospy
import I2C_LCD_driver as LCD
import numpy as np
from geometry_msgs.msg import Pose
from master_msgs_iele3338.srv import AckService
from master_msgs_iele3338.srv import StartService, StartServiceResponse
from master_msgs_iele3338.srv import EndService

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

#Constantes que indican los estados de la raspberry
estadoInicio = ["Hola mundo","R. Olivaw V 1.0"];
estadoPreAck = ["Preparado","Enviando ACK"];
estadoPreparado = ["Listo","Arranque"];
estadoMovimiento = ["Moviendose","Runnnn!!"];
estadoParar = ["Detenido","Completado"];

#Parametros necesarios para la conexion con el master
group_number = 5;
IP="157.253.210.55";

#Inicializacion de pantalla lcd
lcd = LCD.lcd()

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

#Contador de paso de los encoders
contadorD=0
contadorI=0

#Velocidad de los motores
velocidadMD=0 #Motor derecha
velocidadMI=0 #Motor izquierdo

#Direccion de movimiento de los motores
direccionD=1
direccionI=1

#Tiempo general del robot
tiempoRobot = 0;

#Constante del sistema de control
k = [0.02,0.4,0.01]

#Variable de umbral de error superado cuando el robot se acerca a una posicion
umbralSuperado = False;

#Variable de umbral de error superado cuando el robot se acerca a una posicion
errorRho = 0.01;

#Vector que contiene los parametros de las ruedas del robot: alpha, beta, r, l
paraRuedaI = [np.pi/2,0,3.5/100,10.0/100]
paraRuedaD = [-np.pi/2,-np.pi,3.5/100,10.0/100]

#Variable que indica si la prueba ya inicio
pruebaIniciada = False

#Funcion handle iniciada cuando se arranca el servicio start_service
def handle_start_service(msg):

	global posicionActual,posicionFinal

	posicionActual = [msg.start.position.x,msg.start.position.y,msg.start.orientation.w];
	posicionFinal = [msg.goal.position.x,msg.goal.position.y,msg.goal.orientation.w];

	for i in range(msg.n_obstacles):
		obstaculos.append([msg.obstacles[i].position.x,msg.obstacles[i].position.y,msg.obstacles[i].orientation.w])

	pruebaIniciada = True;

	return StartServiceResponse()

#Funcion encargada de enviar la peticion de servicio para finalizar la prueba una vez se alcanzo el objetivo
def terminarServicio(msg):
	end_Request = rospy.ServiceProxy('end_service', EndService)
	EndRobot = end_Request(msg)
	rospy.loginfo(EndRobot)

#Funcion encargada de convertir la respuesta de los encodores en velocidades angulares
def actualizarPosicionActual():
	global contadorD, contadorI, EstadoAactualD, EstadoAanteriorD, EstadoBactualD, tiempoRobot, EstadoAactualI, EstadoAanteriorI, EstadoBactualI, direccionD, direccionI

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

		velocidadMDSensada = direccionD*(contadorD*100/numeroPulsosVuelta)*(2*np.pi)
		velocidadMISensada = direccionI*(contadorI*100/numeroPulsosVuelta)*(2*np.pi)

		tiempoRobot = time.time()

		print("ConD: {}, VelD:{}, ConI: {}, VelDI:{}\n".format(contadorD,velocidadMDSensada,contadorI,velocidadMDSensada))
		
		contadorD=0
		contadorI=0

#Obtiene la posicion del robot en coordenadas polares rho, alpha, beta a partir de la entrada en posicion cartesianas
def obtenerPosicionPol(puntoFinal):
	global umbralSuperado, errorRho, pasoRuta

	rho = np.sqrt((puntoFinal[0]-posicionActual[0])**2+(puntoFinal[1]-posicionActual[1])**2)

	if rho <= errorRho:
		#pasoRuta = pasoRuta + 1;
		#if pasoRuta == len(rutaObjetivo)-1:
		#	errorRho = 0.05
		#	print("De {} a ultimo {}".format(rutaObjetivo[pasoRuta-1],rutaObjetivo[pasoRuta]))
		#elif pasoRuta == len(rutaObjetivo):
		#	umbralSuperado = True
		#	pasoRuta = pasoRuta - 1;
		#else:
		#	print("De {} a {}".format(rutaObjetivo[pasoRuta-1],rutaObjetivo[pasoRuta]))
		umbralSuperado = True

	if not umbralSuperado:
		terminarServicio(np.random.randint(1000,9999))
		alpha = -posicionActual[2]+np.arctan2((puntoFinal[1]-posicionActual[1]),(puntoFinal[0]-posicionActual[0]))
	else:
		alpha = 0;
		rho = 0;

	beta = -alpha-posicionActual[2]

	return np.asarray([rho,alpha,beta])

#Funcion encargada de determinar las velocidades de los motores en cinametica inversa a partir de una ley de control
def calcularCinematicaRobot(puntoFinal):
	global velocidadMD, velocidadMI, paraRuedaD, paraRuedaI
	
	#Obtencion del error de posicion en coordenadas polares
	posPol = np.asarray([0,0,puntoFinal[2]])-obtenerPosicionPol(puntoFinal)

	#Ley de control aplicada para encontrar v y w
	vecVelLinearAngular = np.asarray([k[0]*posPol[0],k[1]*posPol[1]+k[2]*posPol[2]])
	
	#A partir de v y w se determina el vector [x',y',theta']
	veloCar = [(vecVelLinearAngular[0])*(np.cos(posicionActual[2])),(vecVelLinearAngular[0])*(np.sin(posicionActual[2])),vecVelLinearAngular[1]]

	#Matriz de transformacion del marco global al local
	matrixR = [[np.cos(posicionActual[2]),np.sin(posicionActual[2]),0],[-np.sin(posicionActual[2]),np.cos(posicionActual[2]),0],[0,0,1]]
	
	#Obtencion de las velocidades aplicadas a cada motor. Cinematica aplicada
	velocidadMD = np.dot([np.sin(paraRuedaD[0]+paraRuedaD[1]),-np.cos(paraRuedaD[0]+paraRuedaD[1]),-(paraRuedaD[3])*(np.cos(paraRuedaD[1]))],np.dot(matrixR,veloCar))/paraRuedaD[2]
	velocidadMI = np.dot([np.sin(paraRuedaI[0]+paraRuedaI[1]),-np.cos(paraRuedaI[0]+paraRuedaI[1]),-(paraRuedaI[3])*(np.cos(paraRuedaI[1]))],np.dot(matrixR,veloCar))/paraRuedaI[2]

#Funcion principal del codigo. Inicia los parametros ante ROS y mantiene este nodo en operacion, indicando que realizar
def main():

	global contador, EstadoAactual, EstadoAanterior, EstadoBactual, EstadoBanterior, tiempoRobot

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

			if pruebaIniciada:
				actualizarPosicionActual();
				calcularCinematicaRobot(posicionFinal);

			rate.sleep()

	except Exception as e:
		raise e


#Condicion inicial para ejecutar el codigo
if __name__=='__main__':
	main()
else:
	print("Error al iniciar el codigo")
