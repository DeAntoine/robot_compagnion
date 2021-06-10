import detect_people as dp
import liaison_serie as ls
#import estimate_head_pose as hpe
from face_detector_yolo import getFaces
from picamera import PiCamera
from picamera.array import PiRGBArray
from estimate_head_pose import estimate_direction
from argparse import ArgumentParser
from multiprocessing import Process, Queue
from mark_detector import MarkDetector
from os_detector import detect_os
from pose_estimator import PoseEstimator
from stabilizer import Stabilizer

import RPi.GPIO as GPIO

import cv2
import time
import serial

'''
Initialisation du programme
'''


GPIO.setmode(GPIO.BOARD) #Use Board numerotation mode
    #GPIO.setwarnings(False) #Disable warnings

#Use pin id for PWM signal
frequence = 50
GPIO.setup(12, GPIO.OUT)
pwm_12 = GPIO.PWM(12, frequence)
pwm_12.start(0)

GPIO.setup(32, GPIO.OUT)
pwm_32 = GPIO.PWM(32, frequence)
pwm_32.start(0)

GPIO.setup(15, GPIO.OUT)
GPIO.output(15, GPIO.HIGH)
print("pouette")


camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate= 10
time.sleep(2)

rawCapture = PiRGBArray(camera, size = (320,240))
#while camera.isOpened():

i = 0
fichier = open("../resultat_etat.txt", "w+") # "a" pour append et "w" pour écraser
fichier_fps = open("../resultat_fps.txt", "w+") # "a" pour append et "w" pour écraser

t0 = time.perf_counter()

serialArduino = serial.Serial('/dev/ttyACM0', 9600)
ang = 30

serialArduino.write(b'g')
compte=1

for frame1 in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    pwm_12.ChangeDutyCycle(0)
    pwm_32.ChangeDutyCycle(0)
    time_start = time.perf_counter()
    curr_time = time.perf_counter()
    if i != 0 :
        fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")
        fichier.write(str(curr_time-t0)+" 0\n")
        compte=compte+1
        fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")

    frame = frame1.array

    #cv2.imshow('img', frame)

    #cv2.waitKey(1)

    #detect faces
    faces = getFaces(frame)

    curr_time = time.perf_counter()
    if i != 0 :
        fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")
        fichier.write(str(curr_time-t0)+" 0\n")
        compte=compte+1
        fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")



    if len(faces) == 0 :

        print("pas de face")

        # Give dir for a human
        dir_people = dp.detect(frame)

        if dir_people != "r":

            # Send it to arduino
            print("une personne a ete detecte et pas son visage")
            print(dir_people)
            #ls.write(to_bytes(dir_people))

        else :

            # deplacement aleatoire
            print("deplacement aleatoire")


    else :

        print("y a une faces")

        #cv2.imshow('img', frame)
        #cv2.waitKey(0)

        # detect head pose
        # ang = hpe.estimate_pose(frame)

        curr_time = time.perf_counter()
        if i != 0 :
            fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")
            fichier.write(str(curr_time-t0)+" 0\n")
            compte=compte+1
            fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")

        #print(faces[0][0], faces[0][2], faces[0][1], faces[0][3])
        
        # Trouver le centre du carré
        xMil = int((faces[0][0] + faces[0][2])/2)
        yMil = int((faces[0][1] + faces[0][3])/2)
    
        print(xMil, yMil)
    
        #image = cv2.circle(frame, (xMil,yMil), radius=3, color=(0, 0, 255), thickness=2)
        #cv2.imshow('img', frame)
        #cv2.waitKey(1)
        
        height, width = image.shape[:2]
        '''
        if xMil < (width/2)-30 :
            print("a gauche !!!")
            
        elif xMil > (width/2)+30:
            print("a droite !!!")
            
        if yMil > height-80 :
            print("trop bas !!")
        elif yMil <  50 :
            print("trop haut !!")
         '''   
        if (width/2)-30 < xMil < (width/2)+30 :
            if  50 < yMil < height-80 :
                print("c'est bon")
                estimate_direction(frame, pwm_12, pwm_32)
                
                
    time_end = time.perf_counter()
    print(time_end-time_start)
    fichier_fps.write(str(t0)+" "+str(time_end-time_start))
    rawCapture.truncate(0)
    i=i+1


    curr_time = time.perf_counter()
    if i != 0 :
        fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")
        fichier.write(str(curr_time-t0)+" 0\n")
        if i == 100 :
            break
        compte=1
        t0 = time.perf_counter()
        fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")



pwm_12.stop()
pwm_32.stop()
GPIO.cleanup()

# Arduino Esclave

    # Récupérer valeur capteur

    # Dire si obstacle ou non


# Arduino Maitre

    # Recuperer direction pour éviter obstacle

    # Recuperer direction pour aller vers l'objectif

    # Se mettre en mouvement vers cette direction

