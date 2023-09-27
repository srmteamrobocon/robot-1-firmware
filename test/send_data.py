#! /bin/python3
import serial
import time

# Replace '/dev/ttyACM0' with the actual port name
ser = serial.Serial('/dev/ttyACM0', 115200)
current_time_ms = int(time.time() * 1000)
while True:
    time_ms = int(time.time() * 1000) - current_time_ms

    # ser.write('m'.encode('utf-8'))
    # dataToSend = f"{time_ms%1000},-222,3333,\n".encode('utf-8')
    # ser.write(dataToSend)

    # response = ser.readline().strip()  # read(3) will read 3 bytes
    # print(f"[{time_ms//1000}:{time_ms%1000:-3d}] ({dataToSend}) {response}")

    ser.write('e'.encode('utf-8'))
    response = ser.readline().strip()  # read(3) will read 3 bytes
    print(f'Recived data {response}')

    time.sleep(1/50)
