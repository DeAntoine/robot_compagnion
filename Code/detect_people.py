import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
 
# initialisation du HOG:
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# ouverture du flux vidéo de la webcam
cap = cv2.VideoCapture('video.mp4')

t1 = time.perf_counter()

for i in range(300):

    # capture image par image
    ret, frame = cap.read()

    # réduction de l'image pour une détection plus rapide
    frame = cv2.resize(frame, (640, 480))
    # passage en noir et blanc, également pour accélerer 
    # la détection
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    
    # détection des personnes dans l'image. 
    # retourne les coordonnées de la boîte encadrant 
    # les personnes détectées
    boxes, weights = hog.detectMultiScale(frame, winStride=(8,8) )

    """
    boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])
    
    for (xA, yA, xB, yB) in boxes:
        # affichages des boîtes sur l'image couleur
        cv2.rectangle(frame, (xA, yA), (xB, yB),(0, 255, 0), 2)
    """
    #plt.imshow(frame)
    #plt.show()

t2 = time.perf_counter()

print("tps pour 300 exec : ",t2-t1)