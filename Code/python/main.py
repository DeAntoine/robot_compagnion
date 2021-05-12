from serial import *

port_serie = Serial(port="/dev/ttyACM0", baudrate=9600, timeout=1, writeTimeout=1)

def send(a):
    port_serie.write(a)

def read():
    if port_serie.isOpen():
        ligne = port_serie.read_line()
        print(ligne)
