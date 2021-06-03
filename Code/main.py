import detect_people as dp
#import liaison_serie as ls
import head_pose_estimation as hpe
from face_detector_yolo import getFaces
from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2
import time

'''
Initialisation du programme
'''

camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate= 10
time.sleep(2)

rawCapture = PiRGBArray(camera, size = (320,240))
#while camera.isOpened():
for frame1 in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    
    frame = frame1.array

    cv2.imshow('img', frame)

    #detect faces
    faces = getFaces(img)



    if len(faces) == 0 :

        print("pas de face")

        # Give dir for a human
        dir_people = dp.detect(frame)

        if dir_people != "r":

            # Send it to arduino
            print(dir_people)
            #ls.write(to_bytes(dir_people))

        else :

            # deplacement aleatoire
            print("deplacement aleatoire")
            exit()

    else :

        print("y a une faces")

        # detect head pose
        ang = hpe.estimate_pose(frame)

        if ang > 50 :

            #ls.write("g")
            print("g")

        elif ang < 40 :

            #ls.write("d")
            print("d")

        else :

            print("deplacer le pointeur")


    rawCapture.truncate(0)

# Arduino Esclave

    # Récupérer valeur capteur

    # Dire si obstacle ou non
    

# Arduino Maitre

    # Recuperer direction pour éviter obstacle

    # Recuperer direction pour aller vers l'objectif

    # Se mettre en mouvement vers cette direction

