import socket
import time
import cv2
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import tensorflow as tf
from tensorflow import keras
import pyttsx3

# Definir la dirección IP y el puerto del servidor remoto
HOST = '10.0.0.1'
PORT = 5000
cap = cv2.VideoCapture(1)
# Configurar la resolución de captura a 1920x1080
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
umbral = 50  # Umbral para binarización
region_an = 100
region_al = 770 #700
#Importar modelo de cnn
cnn = keras.models.load_model('X:/Archivos/Documentos/Trabajo/UTPL/Investigación control de disturbios/Código Python/Seguidor de línea CNN/cnn_modelo.keras')

# Crear una ventana para mostrar el video en tiempo real
cv2.namedWindow("Video", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Video", region_an, region_al)

# Posición en X y Y en coordenadas del robot (cambian si la posición inicial cambia) (el valor de z dependerá de la aplicación)
x_r = 0
y_r = 500

#Configuración del pyttsx3 para text to speech
engine=pyttsx3.init(driverName='sapi5') #sapi5 para windows, nsss para MacOS y eSpeak para cualquier otro
voces=engine.getProperty('voices')
engine.setProperty('voice',voces[1].id) #voces[0] para voz en inglés y voces[1] para voz en español
engine.setProperty('rate',133) #velocidad de la voz

def capturar(r_an, r_al):
    ret, frame = cap.read()
    imagen_gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convierte la imagen a escala de grises
    _, imagen_binaria = cv2.threshold(imagen_gris, umbral, 255, cv2.THRESH_BINARY)  # Binariza la imagen
    altura, ancho = np.shape(imagen_binaria) # Altura y ancho de la fotografía realizada
    reg_ancho = r_an # Ancho de la región de interés
    reg_altura = r_al # Altura de la región de interés
    reg_x = int(ancho/2) - (reg_ancho//2) # Coordenada x de la región de interés
    reg_y = int(altura/2) - (reg_altura//2) # Coordenada y de la región de interés
    reg = imagen_binaria[reg_y:reg_y+reg_altura, reg_x:reg_x+reg_ancho] # Obtención de la región de interés
    return reg

def escribir(texto):
    texto = '$' + texto + '\r\n'
    comando = texto.encode('utf-8')
    sock.sendall(comando)
    data = sock.recv(1024)
    # Imprimir la respuesta
    print('Respuesta recibida del servidor:', data.decode())
    time.sleep(3)

def comando(texto):
    comando = ('$Execute,"' + texto + '"\r\n').encode('utf-8')
    sock.sendall(comando)
    data = sock.recv(1024)
    # Imprimir la respuesta
    print('Ejecución de comando ' + texto + ":", data.decode())

def punto_comienzo():
    comando("Move XY("+str(x_r)+","+str(y_r)+",180,90,0,180)") # 180 en Z normalmente

def mover_XY(valor_X,valor_Y):
    comando("Move Here +X(" + str(valor_X) +") +Y(" + str(valor_Y) + ")")

def leer_variable(imagen):
    altura, ancho = np.shape(imagen)
    total_pixeles_negros = np.sum(imagen == 0)
    total_pixeles_superior = np.sum(imagen[:altura//2, :] == 0)
    total_pixeles_inferior = np.sum(imagen[altura//2:, :] == 0)    
    diferencia=total_pixeles_superior-total_pixeles_inferior
    return diferencia,total_pixeles_negros
    
def cnn_seguimiento():
    while True:
        # Capturar imagen binarizada y recortada
        imagen = capturar(region_an, region_al)
        
        # Calcular la diferencia de píxeles negros (variable objetivo)
        diferencia, total_p = leer_variable(imagen)
        
        if total_p==0:
           print("No se detectaron píxeles negros. Ejecución terminada.")
           break
       
        # Predecir las salidas X y Y de acuerdo a la imagen obtenida
        [[salida_X,salida_Y]]=cnn.predict((np.expand_dims(imagen, axis=-1).reshape(1, 770, 100, 1))/255)
        
        mover_XY(salida_X,salida_Y)
                  
        cv2.imshow("Video", imagen)
        
        # Actualizar la ventana de visualización y esperar 1ms
        # Si se presiona la tecla 'q', salir del bucle
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def arrancar():
    escribir('Login')
    escribir('SetMotorsOn,0')
    comando("If Error On Then Reset")
    comando('Accel 10,10')
    comando('AccelS 100,100')
    comando('Speed 50')
    comando('SpeedS 3000')
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
    aux = ""
    while aux != "y":
        imagen = capturar(region_an, region_al)
        cv2.imshow("Video", imagen)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        aux = input("¿Empezar? ")
    print("Comenzando ejecución...")
    cnn_seguimiento()
    engine.say("Ejecución terminada. Volviendo a su posición inicial.")
    engine.runAndWait()
    comando("Pulse 0,0,0,0,0,0")
except socket.timeout:
    print('Tiempo de espera agotado para la conexión')
except ConnectionRefusedError:
    print('La conexión fue rechazada por el host remoto')
except Exception as e:
    print('Se produjo un error durante la conexión:', str(e))
finally:
    cap.release()
    cv2.destroyAllWindows()