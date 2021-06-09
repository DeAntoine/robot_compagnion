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
fichier = open("../resultat_etat.txt", "w+") # "a" pour append et "w" pour écraser
t0 = time.perf_counter()

serialArduino = serial.Serial('/dev/ttyACM0', 9600)
ang = 30

serialArduino.write(b'g')


for frame1 in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    time_start = time.perf_counter()
    t1 = time.perf_counter()
    if i != 0 :
        fichier.write(str(t1-t0)+" 1\n")
        fichier.write(str(t1-t0)+" 0\n")
        fichier.write(str(t1-t0)+" 2\n")


    frame = frame1.array

    cv2.imshow('img', frame)

    cv2.waitKey(5)



    if i != 0 :
        t2 = time.perf_counter()
        fichier.write(str(t2-t0)+" 2\n")
        fichier.write(str(t2-t0)+" 0\n")
        fichier.write(str(t2-t0)+" 3\n")

    #detect faces
    faces = getFaces(frame)

    if len(faces) == 0 :

        print("pas de face")

        # Give dir for a human
        dir_people = dp.detect(frame)
        t3 = time.perf_counter()

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

        t4 = time.perf_counter()
        if i != 0 :
            fichier.write(str(t4-t0)+" 3\n")
            fichier.write(str(t4-t0)+" 0\n")
            fichier.write(str(t4-t0)+" 4\n")

        if(ang == 60):
            ang = 30
        else :
            ang = 60

        if ang > 50 :

            serialArduino.write(b'd')
            serialArduino.write(b'e')

            print("g")

        elif ang < 40 :

            serialArduino.write(b'd')
            serialArduino.write(b'e')
            print("d")

        else :

            print("deplacer le pointeur")

    time_end = time.perf_counter()
    print(time_end-time_start)
    rawCapture.truncate(0)
    t4 = time.perf_counter()
    i=i+1
    if i != 1 :
        if len(faces) == 0 :
            fichier.write(str(t4-t0)+" 3\n")
        else :
            fichier.write(str(t4-t0)+" 4\n")
    else :
        t0 = time.perf_counter()

    fichier.write(str(t4-t0)+" 0\n")
    if i == 2 :
        break
    fichier.write(str(t4-t0)+" 1\n")

# Arduino Esclave

    # Récupérer valeur capteur

    # Dire si obstacle ou non


# Arduino Maitre

    # Recuperer direction pour éviter obstacle

    # Recuperer direction pour aller vers l'objectif

    # Se mettre en mouvement vers cette direction

