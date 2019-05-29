#!/usr/bin/env python
#Librerias necesarias para la ejecucion del codigo

import rospy
import cv2, time
from PIL import Image
import PIL.ImageOps
import numpy as np
from _overlapped import none
from std_msgs.msg import Int32

realizarCaptura = False
numeroCamara = None
#Variable que representa el estado del robot
RobotStatus = Int32()
RobotStatus.data = 0

#Metodo callback llamado al haber una actualizacion en el topico RobotStatus
def callbackRobotStatus(msg):
    global realizarCaptura, numeroCamara 
    if msg.data == 2:
        try:
            numeroCamara = camara()
        except Exception as e:
            raise e
            
        if numeroCamara != None:
            RobotStatus.data = 3
    else:
         realizarCaptura = False

#Funcion encargada de realiza la captura de la camara
def camara():
    global realizarCaptura
    numero=None
    ##Toma de la foto
    video = cv2.VideoCapture

    contador = 0
    while True:
        contador = contador + 1
        check,frame = video.read()

        key = cv2.waitKey(1)
        #Espera unos 3 seg antes de tomar foto para asegurar calidad imagen
        if key == ord("q") or contador > 20:
            cv2.imwrite("captura.png",frame);
            break;
    video.release()

    time.sleep(2) #Espera 2 segundos antes de seguir
    ##Lectura de la imagen


    imagen = cv2.imread("captura.png",cv2.IMREAD_GRAYSCALE)
    im_pil = Image.fromarray(imagen)

    #imagen = Image.open("escrito.jpg")
    inv_pil_imagen = PIL.ImageOps.invert(im_pil)

    inv_imagen = np.asarray(inv_pil_imagen)

    #print(inv_imagen)

    h_min = 5000
    h_max = 0

    v_min = 1000
    v_max = 0

    filas = inv_imagen.shape[0]
    columnas = inv_imagen.shape[1]

    for i in range(filas):
        for j in range(columnas):
            if (inv_imagen[i][j] > 150) and j<h_min :
                h_min = j

    for i in range(filas):
        for j in range(columnas):
            if (inv_imagen[i][j] > 150) and j>h_max :
                h_max = j

    for i in range(filas):
        for j in range(columnas):
            if (inv_imagen[i][j] > 150) and i<v_min :
                v_min = i

    for i in range(filas):
        for j in range(columnas):
            if (inv_imagen[i][j] > 150) and i>v_max :
                v_max = i

    print(v_max,v_min)
    print(h_max,h_min)
    #Cortado de la imagen
    #%%


    adi = 4-((h_max-h_min)%4)
    print(adi)

    inv_cropped = inv_imagen[v_min:v_max,h_min:h_max]


    #Extraccion de los puntos limites de los numeros


    filas = inv_cropped.shape[0]
    columnas = inv_cropped.shape[1]
    vector = []
    hay_blanco = False


    for i in range(columnas):
        hay_blanco = False
        for j in range(filas):
            if inv_cropped[j][i] > 105:
                hay_blanco = True

        if hay_blanco == True:
            vector.append(1)
        elif hay_blanco == False:
            vector.append(0)

    indices = np.asarray(vector)

    pos =[]

    for i in range(len(indices)):
        i = i+1
        #Hacer que empiece en 1
        if (indices[i]-indices[i-1])!=0:
            pos.append(i)

        if i==len(indices)-1:
            break

    pos = np.asarray(pos)
    print(pos)

    imagen1 = inv_cropped[:,0:pos[0]]
    imagen2 = inv_cropped[:,pos[1]:pos[2]]
    imagen3 = inv_cropped[:,pos[3]:pos[4]]
    imagen4 = inv_cropped[:,pos[5]:]
    #imagen5 = inv_cropped[:,pos[7]:pos[8]]
    #imagen6 = inv_cropped[:,pos[9]:]
    print('El primer num cortado es')
    #print(imagen1)

    #cv2.imshow("parte1",imagen1)
    #cv2.imshow("Parte2",imagen2)
    #cv2.imshow("parte3",imagen3)
    #cv2.imshow("Parte4",imagen4)
    #cv2.waitKey(0)



    #Transformacion de digitos para que sean del tamano correcto

    #Para modificar el tamano de la imagen se usa el paquete image de PIL
    #Luego hay que transf las iamgenes de array a pil

    im1_pil = Image.fromarray(imagen1)
    im2_pil = Image.fromarray(imagen2)
    im3_pil = Image.fromarray(imagen3)
    im4_pil = Image.fromarray(imagen4)

    #im1_pil.show()



    width = 20
    height = 20

    im1_final_pi = im1_pil.resize((width, height), Image.ANTIALIAS)
    im2_final_pi = im2_pil.resize((width, height), Image.ANTIALIAS)
    im3_final_pi = im3_pil.resize((width, height), Image.ANTIALIAS)
    im4_final_pi = im4_pil.resize((width, height), Image.ANTIALIAS)
    #imTrans.show()


    im1_final = np.asarray(im1_final_pi)
    im2_final = np.asarray(im2_final_pi)
    im3_final = np.asarray(im3_final_pi)
    im4_final = np.asarray(im3_final_pi)

    #cv2.imshow("num 3",im3_final)
    #cv2.waitKey(0)

    #generacion de unico vector test_digits

    test_digits =[]
    test_digits.append(im1_final)
    test_digits.append(im2_final)
    test_digits.append(im3_final)
    test_digits.append(im4_final)

    #print(test_digits[0])
    print('\n')

    #Ya teniendo las imagenes vectorizadas, se entrena el modelo y se halla la mejor aproximacion


    digits = cv2.imread("digits.png",cv2.IMREAD_GRAYSCALE)


    rows = np.vsplit(digits,50) #Divide la imagen en 50 partes
    cells = []

    for row in rows:
        row_cells = np.hsplit(row,50)
        #cv2.imshow("row 0", row_cells[0])
        for cell in row_cells:
            #Vectorizacion de la imagen
            cell = cell.flatten()
            cells.append(cell)

    #Transformacion de las celdas al formato correcto
    cells = np.array(cells,dtype=np.float32)


    #Entrenamiento con los labels correctos
    k = np.arange(10)

    #En la imagen de entrenamiento cada numero se repite 250 luego el label tambien se repite 250 veces
    cells_labels = np.repeat(k,250)


    # Extracion y flatten de las imagenes

    test_cells = []

    for d in test_digits:
        d = d.flatten()
        test_cells.append(d)

    test_cells = np.array(test_cells,dtype=np.float32)

    # KNN
    #Creacion del algoritmo knn (k nearest neighbors)

    knn = cv2.ml.KNearest_create()
    knn.train(cells,cv2.ml.ROW_SAMPLE,cells_labels)
    ret, result, neighbours, dist = knn.findNearest(test_cells, k=10)

    print('El resultado es:')
    print(result)


    factor = 1
    for i in range(result):

        if i==0:
            factor = 1000
        elif i==1:
            factor = 100
        elif i==2:
            factor = 10
        elif i = 3:
            factor = 1

        numero = numero + factor*result[i]

    return numero
#cv2.imshow("num 3",im3_final)
#cv2.waitKey(0)


def main():

    global pubRobotStatus

    try:
        rospy.init_node('Robot5_Camara', anonymous=False)

        #Publica el estado cero 0 en el topico RobotStatus - Este estado representa que el robot no se le ha ordenado iniciar
        pubRobotStatus = rospy.Publisher('RobotStatus',Int32,queue_size=10)

        rospy.Subscriber("RobotStatus",Int32,callbackRobotStatus)

        rate = rospy.Rate(1000)

        while not rospy.is_shutdown():
            if RobotStatus == 2:
                camara();
            rate.sleep()

    except Exception as e:
        raise e


#Condicion inicial para ejecutar el codigo
if __name__=='__main__':
    main()
else:
    print("Error al iniciar el codigo")