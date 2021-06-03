import detect_people as dp
#import liaison_serie as ls
import head_pose_estimation as hpe
from face_detector_yolo import getFaces
import face_landmarks as fl
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

landmark_model = fl.get_landmark_model()

rawCapture = PiRGBArray(camera, size = (320,240))
#while camera.isOpened():
for frame1 in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    frame = frame1.array

    #detect faces
    faces = getFaces(frame)

    marks = fl.detect_marks(frame, landmark_model, face)

    fl.draw_marks(frame, marks)

    cv2.imshow('img', frame)

    rawCapture.truncate(0)


def test_dp(cap):
    t1 = time.perf_counter()
    for i in range(100):
        ret, frame = cap.read()

        dir = dp.detect(frame)
        #print(dir)
    t2 = time.perf_counter()
    print(t2-t1)
