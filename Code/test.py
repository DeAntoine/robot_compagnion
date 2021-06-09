import detect_people as dp
#import liaison_serie as ls
import head_pose_estimation as hpe
from face_detector_yolo import getFaces
import face_landmarks as fl
from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2
import time
import servo as s
from headpose import PoseEstimator

'''
Initialisation du programme
'''

camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate= 10
time.sleep(2)

landmark_model = fl.get_landmark_model()

pin1 = 17
pin2 = 27
#s.set_gpio(pin1)
#s.set_gpio(pin2)

rawCapture = PiRGBArray(camera, size = (320,240))
#while camera.isOpened():
i = 0
fichier = open("../resultat.txt", "w") # "a" pour append et "w" pour écraser
t0 = time.perf_counter()
for frame1 in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):


    if i != 0 :
        t1 = time.perf_counter()
        fichier.write(str(t1-t0)+" 1\n")
        fichier.write(str(t1-t0)+" 0\n")
        fichier.write(str(t1-t0)+" 2\n")

    frame = frame1.array

    #detect faces
    faces = getFaces(frame)
    if i != 0 :
        t2 = time.perf_counter()
        fichier.write(str(t2-t0)+" 2\n")
        fichier.write(str(t2-t0)+" 0\n")
        fichier.write(str(t2-t0)+" 3\n")

    for face in faces :

        print("face detected")

        marks = fl.detect_marks(frame, landmark_model, face)

        fl.draw_marks(frame, marks)

        break

    ang = hpe.estimate_pose(frame)

    print(ang)

    cv2.imshow('image', frame)

    cv2.waitKey(1)
    i = i +1
    if i != 0 :
        t3 = time.perf_counter()
        if i != 1 :
            print(str(1/(t3-t1))+" fps")
            fichier.write(str(t3-t0)+" 3\n")
        fichier.write(str(t3-t0)+" 0\n")
        fichier.write(str(t3-t0)+" 1\n")

    rawCapture.truncate(0)

    if i >= 3:
        break


est = PoseEstimator()  #load the model
rawCapture = PiRGBArray(camera, size = (320,240))

for frame1 in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    frame = frame1.array
    roll, pitch, yawn = est.pose_from_image(frame)  # estimate the head pose
    est.plot_face_detection_marks(frame)  # plot the image with the face detection marks
    rawCapture.truncate(0)

#fichier.close()

def test_dp(cap):
    t1 = time.perf_counter()
    for i in range(100):
        ret, frame = cap.read()

        dir = dp.detect(frame)
        #print(dir)
    t2 = time.perf_counter()
    print(t2-t1)
