import detect_people as dp
import liaison_serie as ls
#import estimate_head_pose as hpe
#from face_detector_yolo import getFaces
from picamera import PiCamera
from picamera.array import PiRGBArray
from estimate_head_pose import estimate_direction, get_face_bis
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

import sys

import matplotlib
matplotlib.use('Agg')

'''
Initialisation du programme
'''
LASER=15
LED_VERTE=11
LED_BLEU=13


is_verbose=False
is_graphe=False
if len(sys.argv) > 1 :
    if '-v' in str(sys.argv) :
        is_verbose=True
    if '-g' in str(sys.argv) :
        is_graphe=True


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

GPIO.setup(LASER, GPIO.OUT)
GPIO.output(LASER, GPIO.HIGH)

GPIO.setup(LED_VERTE, GPIO.OUT)
GPIO.output(LED_VERTE, GPIO.LOW)

GPIO.setup(LED_BLEU, GPIO.OUT)
GPIO.output(LED_BLEU, GPIO.LOW)


print("pouette")
"""
while True :
    pwm_12.ChangeDutyCycle(8.25)
    pwm_32.ChangeDutyCycle(8.25)
"""
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

#serialArduino = serial.Serial('/dev/ttyACM0', 9600)
ang = 30

compte=1

found = False
count = 0

#pwm_12.ChangeDutyCycle(0)
#pwm_32.ChangeDutyCycle(0)

for frame1 in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    GPIO.output(LED_BLEU, GPIO.HIGH)
    pwm_12.ChangeDutyCycle(0)
    pwm_32.ChangeDutyCycle(0)
    if is_graphe :
        time_start = time.perf_counter()
        curr_time = time.perf_counter()
        if i != 0 :
            fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")
            fichier.write(str(curr_time-t0)+" 0\n")
            compte=compte+1
            fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")

    frame = frame1.array

    #detect faces
    #faces = getFaces(frame)
    faces = get_face_bis(frame)

    #if len(faces) == 0 :
    if faces is None:
        if is_verbose :
            print("pas de face")

        count = count + 1
       
        # Give dir for a human
        dir_people = dp.detect(frame)

        if dir_people != "r":
            # Send it to arduino
            if is_verbose :
                print("une personne a ete detecte et pas son visage")
                print(dir_people)
            ls.write(dir_people)

        else :

            # deplacement aleatoire
            if is_verbose :
                print("deplacement aleatoire")
            ls.write('z')
            if found == True :
                print("trouve mais plus detecte, count = ", count)
                if count > 3 :
                    #print("deplacement aleatoire")
                    ls.write('z')
                    found = False
            else:
                print("deplacement aleatoire")
                ls.write('z')


    else :
        if is_verbose :
            print("y a une faces")

        print("y a une faces")
           
            
        #print("/tfaces[0] : ", faces[0])
        #print("/tfaces[3] : ", faces[3])
        cv2.rectangle(frame,(faces[0], faces[1]),(faces[2], faces[3]),(0,255,0),3)
        cv2.imshow('img', frame)
        cv2.waitKey(1)

        # detect head pose
        # ang = hpe.estimate_pose(frame)
        if is_graphe :
            curr_time = time.perf_counter()
            if i != 0 :
                fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")
                fichier.write(str(curr_time-t0)+" 0\n")
                compte=compte+1
                fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")

        # Trouver le centre du carré
        #xMil = int((faces[0][0] + faces[0][2])/2)
        #yMil = int((faces[0][1] + faces[0][3])/2)
    
        xMil = int((faces[0] + faces[2])/2)
        yMil = int((faces[1] + faces[3])/2)
        #image = cv2.circle(frame, (xMil,yMil), radius=3, color=(0, 0, 255), thickness=2)
        #cv2.imshow('img', frame)
        #cv2.waitKey(1)
        
        height, width = frame.shape[:2]
        #largeur_tete = int((faces[0][2]-faces[0][0]))
        #longueur_tete = int((faces[0][3]-faces[0][1]))
        
        largeur_tete = int((faces[2]-faces[0]))
        longueur_tete = int((faces[3]-faces[1]))
        
        
        GPIO.output(LED_VERTE, GPIO.LOW)

        if is_verbose :
            #print("\t\tlargeur tete : ",largeur_tete)
            print("\t\tlongueur tete : ",longueur_tete)
        
        if longueur_tete < 50 :
            ls.write('a')
            
        elif longueur_tete > 90 :
            ls.write('r')
        
        elif xMil < (width/2)-30 :
            ls.write('g')
            
        elif xMil > (width/2)+30:
            ls.write('d')
        else :
            found = True
            count = 0
            ls.write('s')
            GPIO.output(LED_VERTE, GPIO.HIGH)
            estimate_direction(frame, pwm_12, pwm_32)

    rawCapture.truncate(0)

    """
    time_end = time.perf_counter()
    print(time_end-time_start)

    i=i+1
        
        if (width/2)-30 < xMil < (width/2)+30 :
            if  50 < yMil < height-80 :
                ls.write('s')
                estimate_direction(frame, pwm_12, pwm_32)
        
    if is_graphe :
        time_end = time.perf_counter()
        print(time_end-time_start)
        rawCapture.truncate(0)
        i=i+1

        curr_time = time.perf_counter()
        if i == 1 :
            t = time.perf_counter()

    curr_time = time.perf_counter()
    if i == 1 :
        t = time.perf_counter()

        if i != 0 :
            fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")
            fichier.write(str(curr_time-t0)+" 0\n")
            if i == 100 :
                break
            compte=1
            t0 = time.perf_counter()
            fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")
            fichier_fps.write(str(t0-t)+" "+str(time_end-time_start)+"\n")

    """

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

