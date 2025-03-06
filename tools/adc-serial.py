import serial
import time

com = serial.Serial("COM10")
print(com.name)
time.sleep(5)
print(com.read(1000))
com.close()