#!/usr/bin/env python3
#-- coding: utf-8 --
import RPi.GPIO as GPIO
import time


#Set function to calculate percent from angle
def angle_to_percent (angle) :
    if angle > 180 or angle < 0 :
        return False

    start = 4
    end = 12.5
    ratio = (end - start)/180 #Calcul ratio from angle to percent

    angle_as_percent = angle * ratio

    return start + angle_as_percent


def set_gpio(gpio_id) :
    GPIO.setmode(GPIO.BOARD) #Use Board numerotation mode
    GPIO.setwarnings(False) #Disable warnings
    GPIO.setup(gpio_id, GPIO.OUT)


def set_servo_angle (gpio_id , angle) :
    frequence = 50
    pwm = GPIO.PWM(gpio_id, frequence)
    pwm.start(angle_to_percent(angle))


def stop_gpio(gpio_id) :
    frequence = 50
    pwm = GPIO.PWM(gpio_id, frequence)
    pwm.stop()
    GPIO.cleanup()
