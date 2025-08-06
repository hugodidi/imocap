import numpy as np
import struct
from bleak import BleakScanner, BleakClient
import asyncio
import pika
import json
import threading
import sys

class BluetoothMultiplexer(threading.Thread):
    def __init__(self, imu_names):
        super().__init__()
        self.imu_names = imu_names
        self.clients = {name: {"client": None, "channel": None} for name in imu_names}
        self.characteristic_uuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # UUID de las notificaciones
        self.conn = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        self.exchange_name = 'topic_BT'
        self.running = True
        self.flagrunning = True

        # Declarar el exchange global para RabbitMQ
        channel = self.conn.channel()
        channel.exchange_declare(exchange=self.exchange_name, exchange_type='topic')
        channel.close()

    async def scan_and_connect(self):
        """Escanea dispositivos BLE y conecta a los dispositivos especificados en imu_names."""
        print("Escaneando dispositivos BLE...")
        devices = await BleakScanner.discover()

        for name in self.imu_names:
            address = None
            for device in devices:
                if device.name == name:
                    address = device.address
                    break

            if address:
                print(f"Conectando a {name} ({address})...")
                client = BleakClient(address)
                try:
                    await client.connect()
                    await asyncio.sleep(1)  # Pausa para estabilizar la conexión

                    # Crear un canal RabbitMQ para este dispositivo
                    channel = self.conn.channel()
                    channel.exchange_declare(exchange=self.exchange_name, exchange_type='topic')

                    # Guardar cliente y canal por dispositivo
                    self.clients[name]["client"] = client
                    self.clients[name]["channel"] = channel

                    # Asocia el callback con el nombre del dispositivo
                    await client.start_notify(
                        self.characteristic_uuid,
                        lambda sender, data, name=name: self.notification_handler(name, data)
                    )
                    print(f"{name} conectado y notificaciones activadas.")
                except Exception as e:
                    print(f"Error conectando a {name}: {e}")
            else:
                print(f"{name} no encontrado en el escaneo.")

    def notification_handler(self, device_name, data):
        """Callback para manejar datos recibidos de las IMUs."""
        try:
            # Desempaquetar los datos de la IMU
            unpacked_data = struct.unpack('>hhhhhhhhhhIbb', data)
            #print(data[7])
            #print (str(unpacked_data))
            # Enviar los datos al exchange de RabbitMQ
            channel = self.clients[device_name]["channel"]
            routing_key = device_name  # Cada IMU utiliza su nombre como routing key
            channel.basic_publish(
                exchange=self.exchange_name,
                routing_key=routing_key,
                body=json.dumps(unpacked_data)
            )
        except Exception as e:
            print(f"Error procesando notificación de {device_name}: {e}")

    async def cleanup(self):
        """Desconecta todos los dispositivos y limpia los recursos."""
        for name, resources in self.clients.items():
            client = resources["client"]
            channel = resources["channel"]

            if client:
                try:
                    await client.stop_notify(self.characteristic_uuid)
                    await client.disconnect()
                    print(f"{name} desconectado.")
                except Exception as e:
                    print(f"Error desconectando {name}: {e}")
                    sys.exit(0)

            if channel:
                try:
                    channel.close()
                    print(f"Canal RabbitMQ de {name} cerrado.")
                except Exception as e:
                    print(f"Error cerrando el canal RabbitMQ de {name}: {e}")
                    sys.exit(0)

    def stop(self):
        """Marca el hilo para detenerse y limpiar recursos."""
        self.flagrunning = False

    def run(self):
        """Método principal del hilo, se encarga de conectar y mantener la ejecución."""
        async def main():
            await self.scan_and_connect()

            while self.running:
                try:
                    if not self.flagrunning:
                        raise KeyboardInterrupt
                    await asyncio.sleep(1)
                except KeyboardInterrupt:
                    await self.cleanup()

        asyncio.run(main())
