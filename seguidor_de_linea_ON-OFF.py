import socket
import time
import cv2
import numpy as np

# Definir la dirección IP y el puerto del servidor remoto
HOST = '10.0.0.1'
#HOST = '192.168.0.1'
PORT = 5000
cap = cv2.VideoCapture(1)
# Configurar la resolución de captura a 1920x1080
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
umbral=100 #Umbral para binarización

# Posición en X y Y en coordenadas del robot (cambian si la posición inicial cambia) (el valor de z dependerá de la aplicación)
x_r=-54
y_r=592

def capturar(cap,umbral):
    ret,frame = cap.read()
    imagen_gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convierte la imagen a escala de grises
    _,imagen_binaria = cv2.threshold(imagen_gris,umbral,255,cv2.THRESH_BINARY)  # Binariza la imagen
    altura,ancho=np.shape(imagen_binaria) #Altura y ancho de la fotografía realizada
    reg_ancho=800 #Ancho de la región de interés
    reg_altura=800 #Altura de la región de interés
    reg_x=int(ancho/2)-(reg_ancho//2) #Coordenada x de la región de interés
    reg_y=int(altura/2)-(reg_altura//2) #Coordenada y de la región de interés
    reg=imagen_binaria[reg_y:reg_y+reg_altura,reg_x:reg_x+reg_ancho] #Obtención de la región de interés
    #cv2.imshow('Imagen binarizada',reg)  # Muestra la imagen binarizada en una ventana
    #cv2.waitKey(0)  # Espera hasta que se presione una tecla
    return reg
def escribir(texto):
    texto='$'+texto+'\r\n'
    comando=texto.encode('utf-8')
    sock.sendall(comando)
    data = sock.recv(1024)
    # Imprimir la respuesta
    print('Respuesta recibida del servidor:', data.decode())
    time.sleep(3)
def comando(texto):
    comando=('$Execute,"'+texto+'"\r\n').encode('utf-8')
    sock.sendall(comando)
    data = sock.recv(1024)
    # Imprimir la respuesta
    print('Ejecución de comando '+texto+":", data.decode())
def punto_comienzo():
    comando ("Move XY(0,500.000,180,90,0,180)") #180 en Z normalmente
def mover_X(valor_X):
    comando("Move Here +X("+str(valor_X)+")")
def mover_Y(valor_Y):
    comando("Move Here +Y("+str(valor_Y)+")")
def estabilizar():
    img=capturar(cap,100)
    altura,ancho=np.shape(img) #Tamaño de la imagen capturada
    mitad_superior=img[:altura//2,:] #División de la imagen en mitades sup/inf
    mitad_inferior=img[altura//2:,:]
    porcentaje_superior=np.mean(mitad_superior == 0)*100 #Porcentaje de píxeles negros en cada mitad
    porcentaje_inferior=np.mean(mitad_inferior == 0)*100
    print("Porcentaje superior:", porcentaje_superior)
    print("Porcentaje inferior:", porcentaje_inferior)
    if porcentaje_superior == 0 and porcentaje_inferior == 0:
        return False,img # Retornar False si no hay píxeles negros
    else:
        if porcentaje_superior > porcentaje_inferior + 5:
            print("Mover hacia la derecha")
            mover_X(1)  # Mover +X (aumentar la coordenada X del robot)
        elif porcentaje_inferior > porcentaje_superior + 5:
            print("Mover hacia la izquierda")
            mover_X(-1) # Mover en -X (disminuir la coordenada X del robot) 
        else:
            print("Avanzar en Y")
            mover_Y(1)  # Avanzar en Y solo 1 mm
        return True,None  # Retornar True si se encontraron píxeles negros
def arrancar():
    escribir('Login')
    escribir('SetMotorsOn,0')
    comando('Accel 10,10')
    comando('AccelS 100,100')
    comando('Speed 50')
    comando('SpeedS 1000')
    comando('SpeedFactor 50')
    comando('Power High')
    comando('HomeSet 0,0,0,0,0,0')
    #comando('Home')
    punto_comienzo()
    comando('Off 14')

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(180)  # Establece un tiempo de espera de 180 segundos para la conexión

try:
    sock.connect((HOST, PORT))
    print('Conexión establecida')
    arrancar()
    aux=""
    while aux!="y":
        aux=input("¿Empezar? ")
    ejecucion=True
    print("Comenzando ejecución...")
    while ejecucion:
        ejecucion,img=estabilizar()
    print("Ejecución finalizada")
    cv2.imshow("Ultima imagen capturada",img)
    cv2.waitKey(0)
except socket.timeout:
    print('Tiempo de espera agotado para la conexión')
except ConnectionRefusedError:
    print('La conexión fue rechazada por el host remoto')
except:
    print('Se produjo un error durante la conexión')
finally:
    cap.release()
    cv2.destroyAllWindows()