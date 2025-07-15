import RPi.GPIO as GPIO
import time
import serial
import smbus
import math
from flask import Flask, request, jsonify

app = Flask(__name__)

# Configuración de los pines GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)  # Motor 1
GPIO.setup(23, GPIO.OUT)  # Motor 2
GPIO.setup(24, GPIO.OUT)  # Motor 3
GPIO.setup(25, GPIO.OUT)  # Motor 4

# Configuración de la comunicación serial con el GPS
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

# Configuración del sensor MPU-6050
bus = smbus.SMBus(1)
MPU6050_ADDRESS = 0x68
MPU6050_REGISTER_ACCEL_XOUT_H = 0x3B
MPU6050_REGISTER_GYRO_XOUT_H = 0x43

# Configuración del sensor BMP180
BMP180_ADDRESS = 0x77
BMP180_REGISTER_AC1 = 0xAA

def leer_gps():
    while True:
        linea = ser.readline().decode().strip()
        if linea.startswith('$GPGGA'):
            # Extraer datos de ubicación
            datos = linea.split(',')
            latitud = float(datos[2])
            longitud = float(datos[4])
            altitud = float(datos[9])
            return latitud, longitud, altitud

def leer_mpu6050():
    # Leer datos de aceleración y giro
    bus.write_byte(MPU6050_ADDRESS, MPU6050_REGISTER_ACCEL_XOUT_H)
    datos = bus.read_i2c_block_data(MPU6050_ADDRESS, MPU6050_REGISTER_ACCEL_XOUT_H, 6)
    aceleracion_x = (datos[0] << 8) | datos[1]
    aceleracion_y = (datos[2] << 8) | datos[3]
    aceleracion_z = (datos[4] << 8) | datos[5]
    return aceleracion_x, aceleracion_y, aceleracion_z

def leer_bmp180():
    # Leer datos de presión y altitud
    bus.write_byte(BMP180_ADDRESS, BMP180_REGISTER_AC1)
    datos = bus.read_i2c_block_data(BMP180_ADDRESS, BMP180_REGISTER_AC1, 22)
    presion = (datos[0] << 8) | datos[1]
    altitud = (datos[2] << 8) | datos[3]
    return presion, altitud

def controlar_motores(motor1, motor2, motor3, motor4):
    GPIO.output(17, motor1)
    GPIO.output(23, motor2)
    GPIO.output(24, motor3)
    GPIO.output(25, motor4)

@app.route('/control', methods=['POST'])
def control():
    datos = request.get_json()
    motor1 = datos['motor1']
    motor2 = datos['motor2']
    motor3 = datos['motor3']
    motor4 = datos['motor4']
    controlar_motores(motor1, motor2, motor3, motor4)
    return jsonify({'mensaje': 'Motores controlados con éxito'})

def main():
    try:
        # Iniciar el servidor Flask
        app.run(host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Programa terminado")

if __name__ == '__main__':
    main()
