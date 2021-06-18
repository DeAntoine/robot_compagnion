import serial

ser = serial.Serial(port="/dev/ttyACM0", baudrate=9600,timeout=1) 
ser.close() 
ser.open() 

if ser.isOpen():
    print("connexion ok")

def read():
    #ser.readline()
    return ser.read()

def write(c):
    #c = 's'
    ser.write(bytes(c.encode()))
