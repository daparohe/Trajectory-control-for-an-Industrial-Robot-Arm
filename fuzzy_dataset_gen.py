import socket
import time
import cv2
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import os
import pandas as pd

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

# Crear una ventana para mostrar el video en tiempo real
#cv2.namedWindow("Video", cv2.WINDOW_NORMAL)
#cv2.resizeWindow("Video", region_an, region_al)

# Posición en X y Y en coordenadas del robot (cambian si la posición inicial cambia) (el valor de z dependerá de la aplicación)
x_r = 0
y_r = 500

#conteo de datos
contador=6816

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

def guardar_imagen(imagen, carpeta_destino, numero_imagen):
    # Crear la carpeta de destino si no existe
    if not os.path.exists(carpeta_destino):
        os.makedirs(carpeta_destino)
    # Guardar la imagen en la carpeta de destino
    flag=cv2.imwrite(carpeta_destino+"/imagen"+str(numero_imagen)+".jpg",imagen)
    #flag=cv2.imwrite("imagen1.jpg",imagen)
    if flag:
        print(f"Imagen {numero_imagen} guardada correctamente")

def guardar_datos(diferencia, x, y, ruta_archivo):
    # Crear un DataFrame con los datos a guardar
    data = {
        "DIFERENCIA": [diferencia],
        "X": [x],
        "Y": [y]
    }
    df = pd.DataFrame(data)
    # Leer el archivo Excel existente si es que ya existe
    if os.path.exists(ruta_archivo):
        df_existente = pd.read_excel(ruta_archivo)
        # Concatenar el DataFrame existente con los nuevos datos
        df = pd.concat([df_existente, df], ignore_index=True)
    # Guardar el DataFrame en el archivo Excel
    df.to_excel(ruta_archivo, index=False)
    print("Datos guardados correctamente en el archivo Excel.")
    
def fuzzy_controller(setpoint, output_minX, output_maxX, output_minY, output_maxY):   
    
    error = ctrl.Antecedent(np.arange(-7000, 7001, 1), 'error')
    outputX = ctrl.Consequent(np.arange(output_minX, output_maxX + 1, 1), 'salidaX')
    outputY = ctrl.Consequent(np.arange(output_minY, output_maxY + 1, 0.5), 'salidaY')

    # Función de membresía triangular
    error['negativo'] = fuzz.trimf(error.universe, [-7000, -7000, 0])
    error['cero'] = fuzz.trimf(error.universe, [-7000, 0, 7000])
    error['positivo'] = fuzz.trimf(error.universe, [0, 7000, 7000])

    outputX['izquierda'] = fuzz.trimf(outputX.universe, [output_minX, output_minX, 0])
    outputX['centro'] = fuzz.trimf(outputX.universe, [output_minX, 0, output_maxX])
    outputX['derecha'] = fuzz.trimf(outputX.universe, [0, output_maxX, output_maxX])
    
    outputY['detener'] = fuzz.trimf(outputY.universe, [output_minY, output_minY, 0])
    outputY['avanzar'] = fuzz.trimf(outputY.universe, [0, output_maxY, output_maxY])

    rule1 = ctrl.Rule(error['negativo'], outputX['derecha'])
    rule2 = ctrl.Rule(error['cero'], outputX['centro'])
    rule3 = ctrl.Rule(error['positivo'], outputX['izquierda'])
    rule4 = ctrl.Rule(error['negativo'], outputY['detener'])
    rule5 = ctrl.Rule(error['cero'], outputY['avanzar'])
    rule6 = ctrl.Rule(error['positivo'], outputY['detener'])

    system = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6])
    control = ctrl.ControlSystemSimulation(system)
    
    global contador
    
    while True:
        # Capturar imagen binarizada y recortada
        imagen = capturar(region_an, region_al)
        
        # Calcular la diferencia de píxeles negros (variable objetivo)
        diferencia, total_p = leer_variable(imagen)
        
        if total_p==0:
           print("No se detectaron píxeles negros. Ejecución terminada.")
           break
       
       # Guardar la imagen capturada
        guardar_imagen(imagen,"C:/Users/dapar/Desktop/im",contador)
        
        # Calcular el error
        valor_error = setpoint - diferencia

        # Pasar el error al controlador de lógica difusa
        control.input['error'] = valor_error
        control.compute()

        # Obtener la salida del controlador de lógica difusa
        salida_X = control.output['salidaX']
        salida_Y = control.output['salidaY']
               
        # Guardar datos de corrección
        guardar_datos(diferencia,salida_X,salida_Y,'X:/Archivos/Documentos/Trabajo/UTPL/Investigación control de disturbios/Código Python/Seguidor de línea CNN/Datos CNN/data.xlsx')
        
        print("Diferencia:",diferencia,"píxeles")
        
        mover_XY(salida_X,salida_Y)
        
        contador=contador+1

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
    aux = ""
    while aux != "y":
        #imagen = capturar(region_an, region_al)
        #cv2.imshow("Video", imagen)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
            #break
        aux = input("¿Empezar? ")
    valor_objetivo=0
    actuador_minX=-1
    actuador_maxX=1
    actuador_minY=0
    actuador_maxY=1
    for b in range(5):
        print("Comenzando ejecución número",b)
        fuzzy_controller(valor_objetivo,actuador_minX,actuador_maxX,actuador_minY,actuador_maxY)
        punto_comienzo()
    #comando("Home")
except socket.timeout:
    print('Tiempo de espera agotado para la conexión')
except ConnectionRefusedError:
    print('La conexión fue rechazada por el host remoto')
except Exception as e:
    print('Se produjo un error durante la conexión:', str(e))
finally:
    cap.release()
    cv2.destroyAllWindows()