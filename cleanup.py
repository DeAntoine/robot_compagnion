import RPi.GPIO as GPIO

# the rest of your code would go here

# when your code ends, the last line before the program exits would be...

GPIO.setmode(GPIO.BOARD) #Use Board numerotation mode

GPIO.setup(12, GPIO.OUT)
GPIO.setup(32, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)

GPIO.cleanup()

# remember, a program doesn't necessarily exit at the last line!
