import serial

ser = serial.Serial(port="/dev/ttyACM0", baudrate=9600,timeout=1) 
ser.close() 
ser.open() 

if ser.isOpen():
    print("ok")

def read():
    #ser.readline()
    return ser.read()

def write(c):
    #ser.write(bytes(c.encode()))
    print(c)
