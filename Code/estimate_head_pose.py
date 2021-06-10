"""Demo code shows how to estimate human head pose.
Currently, human face is detected by a detector from an OpenCV DNN module.
Then the face box is modified a little to suits the need of landmark
detection. The facial landmark detection is done by a custom Convolutional
Neural Network trained with TensorFlow. After that, head pose is estimated
by solving a PnP problem.
"""
from argparse import ArgumentParser
from multiprocessing import Process, Queue
from picamera import PiCamera
from picamera.array import PiRGBArray

import cv2
import numpy as np

from mark_detector import MarkDetector
from os_detector import detect_os
from pose_estimator import PoseEstimator
from stabilizer import Stabilizer

import RPi.GPIO as GPIO
import time

print("OpenCV version: {}".format(cv2.__version__))

# multiprocessing may not work on Windows and macOS, check OS for safety.
detect_os()

CNN_INPUT_SIZE = 128

# Take arguments from user input.
parser = ArgumentParser()
parser.add_argument("--video", type=str, default=None,
                    help="Video file to be processed.")
parser.add_argument("--cam", type=int, default=None,
                    help="The webcam index.")
args = parser.parse_args()


def get_face(detector, img_queue, box_queue):
    """Get face from image queue. This function is used for multiprocessing"""
    while True:
        image = img_queue.get()
        box = detector.extract_cnn_facebox(image)
        box_queue.put(box)

def angle_to_percent (angle) :
    if angle > 180 or angle < 0 :
        return False

    start = 4
    end = 12.5
    ratio = (end - start)/180 #Calcul ratio from angle to percent

    angle_as_percent = angle * ratio

    return start + angle_as_percent


def main():
    """MAIN"""
    # Video source from webcam or video file.
    video_src = args.cam if args.cam is not None else args.video
    if video_src is None:
        print("Warning: video source not assigned, default webcam will be used.")
        video_src = 0
        
    # Introduce mark_detector to detect landmarks.
    mark_detector = MarkDetector()
    
    
    # Read frame, crop it, flip it, suits your needs.
    camera = PiCamera()
    camera.resolution = (320, 240)
    camera.framerate= 10
    time.sleep(2)

    rawCapture = PiRGBArray(camera, size = (320,240))
    frame = 0
    for frame1 in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        frame = frame1.array
        break
         
    
    # Setup process and queues for multiprocessing.
    img_queue = Queue()
    box_queue = Queue()
    img_queue.put(frame)
    box_process = Process(target=get_face, args=(
        mark_detector, img_queue, box_queue,))
    box_process.start()

    # Introduce pose estimator to solve pose. Get one frame to setup the
    # estimator according to the image size.
    height, width = frame.shape[:2]
    pose_estimator = PoseEstimator(img_size=(height, width))

    # Introduce scalar stabilizers for pose.
    pose_stabilizers = [Stabilizer(
        state_num=2,
        measure_num=1,
        cov_process=0.1,
        cov_measure=0.1) for _ in range(6)]

    tm = cv2.TickMeter()

        
    rawCapture = PiRGBArray(camera, size = (320,240))

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

    GPIO.setup(15, GPIO.OUT)
    GPIO.output(15, GPIO.HIGH)
    print("pouette")


#while camera.isOpened():
    for frame1 in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

        frame = frame1.array
       
        # Feed frame to image queue.
        img_queue.put(frame)

        # Get face from box queue.
        facebox = box_queue.get()

        if facebox is not None:
          
            print("face detectedddd")
          
            # Detect landmarks from image of 128x128.
            face_img = frame[facebox[1]: facebox[3],
                             facebox[0]: facebox[2]]
            face_img = cv2.resize(face_img, (CNN_INPUT_SIZE, CNN_INPUT_SIZE))
            face_img = cv2.cvtColor(face_img, cv2.COLOR_BGR2RGB)

            tm.start()
            marks = mark_detector.detect_marks(face_img)
            tm.stop()

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

            # Stabilize the pose.
            steady_pose = []
            pose_np = np.array(pose).flatten()
            for value, ps_stb in zip(pose_np, pose_stabilizers):
                ps_stb.update([value])
                steady_pose.append(ps_stb.state[0])
            steady_pose = np.reshape(steady_pose, (-1, 3))
            #print(steady_pose)
            # Uncomment following line to draw pose annotation on frame.
            # pose_estimator.draw_annotation_box(frame, pose[0], pose[1], color=(255, 128, 128))

            # Uncomment following line to draw stabile pose annotation on frame.
            pose_estimator.draw_annotation_box(frame, steady_pose[0], steady_pose[1], pwm_12,pwm_32, color=(128, 255, 128))

            # Uncomment following line to draw head axes on frame.
            # pose_estimator.draw_axes(frame, steady_pose[0], steady_pose[1])

        # Show preview.
        #cv2.imshow("Preview", frame)
        #if cv2.waitKey(10) == 27:
            #break
            
        rawCapture.truncate(0)

    # Clean up the multiprocessing process.
    box_process.terminate()
    box_process.join()
    pwm_12.stop()
    pwm_32.stop()
    GPIO.cleanup()
  
mark_detector = MarkDetector()
  
def estimate_direction(frame, facebox, pwm_12, pwm_32):
         
    print("estimate direction")
    height, width = frame.shape[:2]
    pose_estimator = PoseEstimator(img_size=(height, width))

    # Introduce scalar stabilizers for pose.
    pose_stabilizers = [Stabilizer(
        state_num=2,
        measure_num=1,
        cov_process=0.1,
        cov_measure=0.1) for _ in range(6)]

    
      
    if facebox is not None:
      
        print("face detected")
        print("face detected")
        print("face detected")
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

        # Stabilize the pose.
        steady_pose = []
        pose_np = np.array(pose).flatten()
        for value, ps_stb in zip(pose_np, pose_stabilizers):
            ps_stb.update([value])
            steady_pose.append(ps_stb.state[0])
        steady_pose = np.reshape(steady_pose, (-1, 3))
        #print(steady_pose)
        # Uncomment following line to draw pose annotation on frame.
        # pose_estimator.draw_annotation_box(frame, pose[0], pose[1], color=(255, 128, 128))

        # Uncomment following line to draw stabile pose annotation on frame.
        pose_estimator.draw_annotation_box(frame, pose[0], pose[1], pwm_12,pwm_32, color=(128, 255, 128))

        # Uncomment following line to draw head axes on frame.
        # pose_estimator.draw_axes(frame, steady_pose[0], steady_pose[1])
   
    # Show preview.
    cv2.imshow("Preview", frame)
    cv2.waitKey(1)


