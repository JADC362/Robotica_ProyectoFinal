#!/usr/bin/env python
import time
import RPi.GPIO as GPIO
import rospy
import I2C_LCD_driver as LCD
import numpy as np
from master_msgs_iele3338.srv import AckService
from master_msgs_iele3338.srv import StartService, StartServiceResponse
from master_msgs_iele3338.srv import EndService
#import lcddriver

from std_msgs.msg import Int32

estadoInicio = ["Hola mundo","R. Olivaw V 1.0"];
estadoPreAck = ["Preparado","Enviando ACK"];
estadoPreparado = ["Listo","Arranque esta joda"];
estadoMovimiento = ["Moviendose","Runnnn!!"];
estadoParar = ["Detenido","Completado"];

IP="157.253.210.55"
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

PWM=18
en2=22
en1=32

PWM2=40
en4=36
en3=38

lcd = LCD.lcd()


EncoderAD=35
EncoderBD=37
EncoderAI=11
EncoderBI=7

numeroPulsosVuelta=442

GPIO.setmode(GPIO.BOARD)

GPIO.setup(en4, GPIO.OUT)
GPIO.setup(PWM2, GPIO.OUT)
GPIO.setup(en3, GPIO.OUT)
GPIO.setup(en2, GPIO.OUT)
GPIO.setup(PWM, GPIO.OUT)
GPIO.setup(en1, GPIO.OUT)

rueda = GPIO.PWM(PWM,100)
rueda.start(100)

rueda2 = GPIO.PWM(PWM2,100)
rueda2.start(100)

GPIO.output(en3,False)
GPIO.output(en4,False)

GPIO.output(en1,False)
GPIO.output(en2,False)

GPIO.setup(EncoderAD, GPIO.IN)
GPIO.setup(EncoderBD, GPIO.IN)
GPIO.setup(EncoderAI,GPIO.IN)
GPIO.setup(EncoderBI,GPIO.IN)

GPIO.input(EncoderAD)
GPIO.input(EncoderBD)
GPIO.input(EncoderAI)
GPIO.input(EncoderBI)

EstadoAactualD=0
EstadoAanteriorD=GPIO.input(EncoderAD)
EstadoBactualD=0

EstadoAactualI=0
EstadoAanteriorI=GPIO.input(EncoderAI)
EstadoBactualI=0

contadorD=0
contadorI=0

velocidadDerecha=0
velocidadIzquierda=0

direccionD=1
direccionI=1

tiempoinicial=0

def handle_start_service(msg):
	rospy.loginfo(msg)
	lcd.lcd_clear()
	lcd.lcd_display_string(estadoPreparado[0], 1)
	lcd.lcd_display_string(estadoPreparado[1], 2)
	mover5()
	end_Request = rospy.ServiceProxy('end_service', EndService)
	numero=np.random.randint(1000,9999)
	rospy.loginfo(numero)
	EndRobot = end_Request(numero)
	rospy.loginfo(EndRobot)
	parar()
	return StartServiceResponse()

def parar():
	GPIO.output(en3, False)
	GPIO.output(en4, False)
	GPIO.output(en1, False)
	GPIO.output(en2, False)
	lcd.lcd_clear()
	lcd.lcd_display_string(estadoParar[0], 1)
	lcd.lcd_display_string(estadoParar[1], 2)
	pass
def mover5():
	lcd.lcd_clear()
	lcd.lcd_display_string(estadoMovimiento[0], 1)
	lcd.lcd_display_string(estadoMovimiento[1], 2)
	GPIO.output(en3, True)
	GPIO.output(en4, False)
	GPIO.output(en1, True)
	GPIO.output(en2, False)
	time.sleep(30)
	pass

def velocidadSg():
	global contadorD,contadorI, EstadoAactualD, EstadoAanteriorD, EstadoBactualD, tiempoinicial, velocidadDerecha,velocidadIzquierda, EstadoAactualI, EstadoAanteriorI, EstadoBactualI, direccionD,direccionI
	EstadoAanteriorD=EstadoAactualD
	EstadoAactualD=GPIO.input(EncoderAD)
	EstadoBactualD=GPIO.input(EncoderBD)

	EstadoAanteriorI = EstadoAactualI
	EstadoAactualI = GPIO.input(EncoderAI)
	EstadoBactualI = GPIO.input(EncoderBI)

	if EstadoAactualD!=EstadoAanteriorD and EstadoAactualD==1:
		#print("Aa: {},Aant: {}, cont:{}".format(EstadoAactual,EstadoAanterior,contador))
		contadorD+=1
		if EstadoAactualD!=EstadoBactualD:
			direccionD=1
		else:
			direccionD=-1

	if EstadoAactualI!=EstadoAanteriorI and EstadoAactualI==1:
		#print("Aa: {},Aant: {}, cont:{}".format(EstadoAactual,EstadoAanterior,contador))
		contadorI+=1
		if EstadoAactualI!=EstadoBactualI:
			direccionI=1
		else:
			direccionI=-1

	delta=time.time()-tiempoinicial
	if delta>=0.01:
		velocidadDerecha=direccionD*(contadorD*100/numeroPulsosVuelta)*(2*np.pi)
		velocidadIzquierda=direccionI*(contadorI*100/numeroPulsosVuelta)*(2*np.pi)
		tiempoinicial=time.time()
		print("velD: {}, RPSD:{}, velI: {}, RPSI:{}\n".format(velocidadDerecha,(contadorD*100)/numeroPulsosVuelta,velocidadIzquierda,(contadorI*100)/numeroPulsosVuelta))
		contadorD=0
		contadorI=0
	pass

def main():
	global contador,EstadoAactual,EstadoAanterior,EstadoBactual,EstadoBanterior,tiempoinicial
	lcd.lcd_display_string(estadoInicio[0], 1)
	lcd.lcd_display_string(estadoInicio[1], 2)
	try:
		rospy.init_node('PruebaMotoresRobot5', anonymous=True)
		rate = rospy.Rate(10)
		rospy.wait_for_service('ack_service')
		robotRequest = rospy.ServiceProxy('ack_service', AckService)
		AckRobot = robotRequest(5, IP)
		lcd.lcd_clear()
		lcd.lcd_display_string(estadoPreAck[0], 1)
		lcd.lcd_display_string(estadoPreAck[1], 2)
		while AckRobot.state == 0:
			AckRobot = robotRequest(5, IP)
			rospy.loginfo("esperando")

		rospy.loginfo(AckRobot)
		start_request = rospy.Service('start_service', StartService, handle_start_service)
		while not rospy.is_shutdown():
			velocidadSg()
			pass
	except Exception as e:
		raise e
		pass


if __name__=='__main__':
	main()
else:
	print("Error al cargar")