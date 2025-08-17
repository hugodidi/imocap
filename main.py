import pika.exceptions
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import math
import numpy as np
from datetime import datetime
from BTClassV2_multiplex import BluetoothMultiplexer
from IMUmqrabbit import IMU
import asyncio
import queue
import time
import pika, sys
import multiprocessing
import socket, json
import subprocess
import os
import psutil 
import winrt
import winrt.windows.foundation
import winrt.windows.foundation.collections
# Configuración de conexión con Unity
host = '127.0.0.1'
port = 25001

def install_rabbitmq(mqrabbit, erlang):
    if not os.path.exists("C:\\Program Files\\RabbitMQ Server"):
        print("Instalando RabbitMQ...")
        subprocess.run([erlang, "/S"], check=True)  # Instala Erlang
        subprocess.run([mqrabbit, "/S"], check=True)  # Instala RabbitMQ
        print("Instalación completada. Reinicia el sistema si es necesario.")
def is_already_running():
    if os.path.exists(LOCK_FILE):
        return True
    open(LOCK_FILE,'w').close()
    return False
def cleanup():
    if os.path.exists(LOCK_FILE):
        os.remove(LOCK_FILE)

cal = 0
qz_prev = 0
# Manejo de Paths para PyInstaller
def get_script_dir(relative_path):
    if getattr(sys, 'frozen', False):  # Detecta si es un ejecutable
        base_path = sys._MEIPASS if hasattr(sys, '__MEISPASS') else os.path.dirname(sys.executable)
    else:
        base_path = os.path.dirname(__file__)
    return os.path.join(base_path, relative_path)

#unity_exe = get_script_dir("250312Windows_exe/My project (3).exe")
installer_mqrabbit = get_script_dir("Install/rabbitmq-server-4.0.6.exe")
installer_erlang = get_script_dir("Install/otp_win64_27.2.4.exe")
install_rabbitmq(installer_mqrabbit, installer_erlang)

appdata_folder = os.path.join(os.getenv("LOCALAPPDATA"), "nMotion")
os.makedirs(appdata_folder, exist_ok=True)
LOCK_FILE = os.path.join(appdata_folder, "script.lock")
IMUnames = ['GROOT_01', 'GROOT_02', 'GROOT_03', 'GROOT_04', 'GROOT_05', 'GROOT_06', 'GROOT_07']

# Función para iniciar el receptor de datos desde RabbitMQ
def start_receiver(binding_keys, stop_event, main_event):
    try:
        # client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        # client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 512)
        time.sleep(5)
        print("Socket con Unity creado correctamente.")
        print("Conectando con Unity...")
        try:
            # client_socket.connect((host, port))
            pass
        except (ConnectionRefusedError, OSError) as e:
            print(f"No se pudo conectar a Unity: {e}")
            return  # Salir si Unity no está disponible
        connection = pika.BlockingConnection(
            pika.ConnectionParameters(host='localhost'))
        channel = connection.channel()
        channel.exchange_declare(exchange='topic_Proc', exchange_type='topic')
        
        result = channel.queue_declare('', exclusive=True)
        queue_name = result.method.queue

        for binding_key in binding_keys:
            channel.queue_bind(exchange='topic_Proc', queue=queue_name, routing_key=binding_key)
        print(' [*] Esperando datos de IMUs...')
        
        def callback(ch, method, properties, body):
            try:
                global cal, stop_event
                if cal == 0:
                    cal += 1
                    print("Calibración finalizada. Puede moverse.")
                
                values = json.loads(body.decode('utf-8'))
                qw,qx,qy,qz = values

                ## Lite message version
                message = f"{qx},{qy},{qz},{qw}\n"
                print(message)

                try:
                    # client_socket.sendall(message.encode())
                    sys.stdout.flush() 
                    time.sleep(0.01) 
                except (BrokenPipeError, ConnectionResetError, OSError) as e:
                   print(f"Error en el socket con Unity: {e}")
                   stop_event.set()
                   raise Exception
            except json.JSONDecodeError:
                print("Error al decodificar")
            except Exception as e:
                print(f"Error inesperado en callback: {e}") 
                stop_event.set()
                sys.exit(1)
        channel.basic_consume(queue=queue_name, on_message_callback=callback, auto_ack=True)
        channel.start_consuming()

    except pika.exceptions.AMQPConnectionError:
        print("No se pudo conectar con MQRabbit")
    except Exception as e:
        stop_event.set()
        print(f"Error en el receptor: {e}")
    finally:
        stop_event.set()
        main_event.set()
        cleanup()
        print("Cerrando MQRabbit")
        try:
            if channel and channel.is_open:
                try:
                    channel.close()
                    print("Canal RabbitMQ cerrado.")
                except Exception as e:
                    print(f"Error cerrando el canal de RabbitMQ: {e}")
        except UnboundLocalError:
            print("El canal de RabbitMQ no fue creado")
        try:   
            if connection and connection.is_open:
                try:
                    connection.close()
                    print("Conexión RabbitMQ cerrada.")
                except Exception as e:
                    print(f"Error cerrando la conexión de RabbitMQ: {e}")
        except UnboundLocalError:
            print("La conexión de RabbitMQ no fue creada")

        # if client_socket:
        #    try:
        #        client_socket.close()
        #        print("Socket con Unity cerrado.")
        #    except Exception as e:
        #        print(f"Error cerrando socket con Unity: {e}")
        return
def proceso_imu(nombre, stop_event):
    try:
        imu = IMU(nombre)
        imu.run()
        while not stop_event.is_set():
            time.sleep(0.1)
    except Exception as e:
        print(f"Error en IMU {nombre}: {e}")
    finally:
        imu.stop()
        print(f"IMU {nombre} terminada")

# def unity_running():
#     for process in psutil.process_iter(attrs=['pid', 'name']):
#         if "Unity" in process.info['name']: 
#             print(f"Unity ya está en ejecución (PID {process.info['pid']}). No se lanzará otra instancia.")
#             return True
#     return False
# def launch_unity():
#     """ Lanza Unity solo si no está ya corriendo """
#     if not unity_running():
#         print("Lanzando Unity...")
#         subprocess.Popen([unity_exe], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
#         time.sleep(5)  # 🟢 Esperar unos segundos para que Unity arranque
#         print("Unity lanzado correctamente.")
#     else:
#         print("Unity ya está en ejecución, no es necesario lanzarlo.")
def main():
    try:
        stop_event = multiprocessing.Event()
        main_event = multiprocessing.Event()
        procesos_imu=[]
        bluetooth_receiver = BluetoothMultiplexer(IMUnames)
        print("Inicializando Bluetooth e IMUs...")
        bluetooth_receiver.start()
        print("Bluetooth inicializado correctamente")
        for imu in IMUnames:
            p = multiprocessing.Process(target=proceso_imu, args=(imu,stop_event))
            p.start()
            procesos_imu.append(p)
        proceso = multiprocessing.Process(target=start_receiver, args=(IMUnames,stop_event, main_event))
        proceso.start()
        main_event.wait()
        proceso.join(timeout=5)

        if proceso.is_alive():
            proceso.terminate()
            proceso.join()
        for p in procesos_imu:
            p.join(timeout=1)
            if p.is_alive():
                p.terminate()
                p.join()

    except Exception as e:
        print(f"Error en main: {e}")
    finally:
        print("Cerrando conexiones...")
        bluetooth_receiver.stop()

        for p in procesos_imu:
            if p.is_alive():
                p.terminate()
                p.join()

        if proceso and proceso.is_alive():
            proceso.terminate()
            proceso.join()
        

        cleanup()
        sys.exit(0)

if __name__ == "__main__":
    
    multiprocessing.freeze_support()

    try:
        print("INICIANDO CALIBRACIÓN. EVITE MOVERSE")
        #launch_unity()
        time.sleep(10)
        if is_already_running():
            sys.exit(0)
        main()
    
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        print("Cerrando procesos...")
        print("Procesos finalizados correctamente.")
        cleanup()
        
