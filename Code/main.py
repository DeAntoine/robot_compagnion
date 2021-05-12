import detect_people as dp
#import liaison_serie as ls
#
import cv2
import time

def test():
    t1 = time.perf_counter()
    for i in range(100):
        ret, frame = cap.read()

        dir = dp.detect(frame)
        #print(dir)
    t2 = time.perf_counter()
    print(t2-t1)

'''
Initialisation du programme
'''

cap = cv2.VideoCapture('video.mp4')

test()
"""
while(1):

    # Get a image
    ret, frame = cap.read()

    # Give dir for a human
    dir_people = dp.detect(frame)

    # Send it to arduino
    ls.write(dir_people)


# Code qui va se lancer au démarrage du raspberry

    # while Pas de visage

        # Recuperer image camera

        # Si visage

            # Trouver direction du visage

        # Sinon

            # Detecter humain

                # En deduire une direction

                # Envoyer direction à Arduino


# Arduino Esclave

    # Récupérer valeur capteur

    # Dire si obstacle ou non
    

# Arduino Maitre

    # Recuperer direction pour éviter obstacle

    # Recuperer direction pour aller vers l'objectif

    # Se mettre en mouvement vers cette direction
"""
