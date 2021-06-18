from argparse import ArgumentParser
from multiprocessing import Process, Queue
from picamera import PiCamera
from picamera.array import PiRGBArray

import cv2
import numpy as np

from mark_detector import MarkDetector
from pose_estimator import PoseEstimator
from stabilizer import Stabilizer

import RPi.GPIO as GPIO
import time

CNN_INPUT_SIZE = 128

mark_detector = MarkDetector()  

def get_face(detector, img_queue, box_queue):
    """Get face from image queue. This function is used for multiprocessing"""
    while True:
        image = img_queue.get()
        box = detector.extract_cnn_facebox(image)
        box_queue.put(box)

def get_face_bis(frame):
    facebox = mark_detector.extract_cnn_facebox(frame)
    return facebox
        
def angle_to_percent (angle) :
    if angle > 180 or angle < 0 :
        return False

    start = 4
    end = 12.5
    ratio = (end - start)/180 #Calcul ratio from angle to percent

    angle_as_percent = angle * ratio

    return start + angle_as_percent

def estimate_direction(frame, pwm_12, pwm_32):
         
    print("estimate direction")
    height, width = frame.shape[:2]
    pose_estimator = PoseEstimator(img_size=(height, width))

    # Introduce scalar stabilizers for pose.
    pose_stabilizers = [Stabilizer(
        state_num=2,
        measure_num=1,
        cov_process=0.1,
        cov_measure=0.1) for _ in range(6)]

    facebox = mark_detector.extract_cnn_facebox(frame)
      
    if facebox is not None:
      
        #print("face detected")
        #print("face detected")
        #print("face detected")
        # Detect landmarks from image of 128x128.
        face_img = frame[facebox[1]: facebox[3],
                         facebox[0]: facebox[2]]
        face_img = cv2.resize(face_img, (CNN_INPUT_SIZE, CNN_INPUT_SIZE))
        face_img = cv2.cvtColor(face_img, cv2.COLOR_BGR2RGB)
        
        marks = mark_detector.detect_marks(face_img)
       
        # Convert the marks locations from local CNN to global image.
        marks *= (facebox[2] - facebox[0])
        marks[:, 0] += facebox[0]
        marks[:, 1] += facebox[1]

        # Uncomment following line to show raw marks.
        #mark_detector.draw_marks(frame, marks, color=(0, 255, 0))

        # Uncomment following line to show facebox.
        #mark_detector.draw_box(frame, [facebox])

        # Try pose estimation with 68 points.
        pose = pose_estimator.solve_pose_by_68_points(marks)
        """
        # Stabilize the pose.
        steady_pose = []
        pose_np = np.array(pose).flatten()
        for value, ps_stb in zip(pose_np, pose_stabilizers):
            ps_stb.update([value])
            steady_pose.append(ps_stb.state[0])
        steady_pose = np.reshape(steady_pose, (-1, 3))
        #print(steady_pose)
        """
        # Uncomment following line to draw pose annotation on frame.
        pose_estimator.draw_annotation_box(frame, pose[0], pose[1], pwm_12,pwm_32,color=(255, 128, 128))

        # Uncomment following line to draw stabile pose annotation on frame.
        #pose_estimator.draw_annotation_box(frame, steady_pose[0], steady_pose[1], pwm_12,pwm_32, color=(128, 255, 128))

        # Uncomment following line to draw head axes on frame.
        #pose_estimator.draw_axes(frame, steady_pose[0], steady_pose[1])
   
    # Show preview.
    #cv2.imshow("Preview", frame)
    #cv2.waitKey(1)


