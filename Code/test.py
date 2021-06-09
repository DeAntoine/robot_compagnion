import detect_people as dp
import liaison_serie as ls
#import estimate_head_pose as hpe
from face_detector_yolo import getFaces
from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2
import time
import serial

'''
Initialisation du programme
'''

camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate= 10
time.sleep(2)

rawCapture = PiRGBArray(camera, size = (320,240))
#while camera.isOpened():

i = 0
fichier = open("../resultat.txt", "w") # "a" pour append et "w" pour écraser
t0 = time.perf_counter()

serialArduino = serial.Serial('/dev/ttyACM0', 9600)
ang = 30
print("test")
serialArduino.write('d')
print("fok")
for frame1 in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    t1 = time.perf_counter()
    if i != 0 :
        fichier.write(str(t1-t0)+" 1\n")
        fichier.write(str(t1-t0)+" 0\n")
        fichier.write(str(t1-t0)+" 2\n")


    frame = frame1.array

    cv2.imshow('img', frame)

    cv2.waitKey(5)

    #detect faces
    faces = getFaces(frame)

    if len(faces) == 0 :

        print("pas de face")

        # Give dir for a human
        dir_people = dp.detect(frame)

        if dir_people != "r":

            # Send it to arduino
            print("une peronne a ete detecte et pas son visage")
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

        if(ang == 60):
            ang = 30
        else :
            ang = 60

        if ang > 50 :

            ls.write(serial.to_bytes('g'))
            print("g")
            #serialArduino.write(to_bytes("g"))

        elif ang < 40 :

            ls.write(serial.to_bytes('d'))
            print("d")
            #serialArduino.write(to_bytes("d"))

        else :

            print("deplacer le pointeur")

    t2 = time.perf_counter()
    print(t2-t1)
    rawCapture.truncate(0)

# Arduino Esclave

    # Récupérer valeur capteur

    # Dire si obstacle ou non


# Arduino Maitre

    # Recuperer direction pour éviter obstacle

    # Recuperer direction pour aller vers l'objectif

    # Se mettre en mouvement vers cette direction

