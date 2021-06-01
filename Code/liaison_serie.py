import serial

ser = serial.Serial(port="/dev/ttyACM0", baudrate=115200,timeout=1) 
ser.close() 
ser.open() 

if ser.isOpen():
    print("ok")

def read():
    #ser.readline()
    return ser.read()

def write(c):
    ser.write(c)