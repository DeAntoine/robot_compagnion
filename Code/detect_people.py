import numpy as np
import cv2
import matplotlib.pyplot as plt
import time

milVertical = 320

# initialisation du HOG:
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

def detect(frame):

    frame = cv2.resize(frame, (640, 480))
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    boxes, weights = hog.detectMultiScale(frame, winStride=(16,16))
    boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])
    
    if np.size(weights) > 0:
        print("on est la")
        max = weights.max()
        index = np.where(weights == max)[0]
        
        if abs(boxes[index][0][0] - milVertical) <= 20 :
            return "c"

        elif boxes[index][0][0] < milVertical-20:
            return "g"

        elif boxes[index][0][0] > milVertical+20:
            return "d"

    return "r"

    cv2.rectangle(frame, (boxes[index][0][0], boxes[index][0][1]), (boxes[index][0][2], boxes[index][0][3]),(0, 255, 0), 2)
    plt.imshow(frame)
    plt.show()
