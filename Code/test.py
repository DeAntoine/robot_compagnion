import detect_people as dp
import liaison_serie as ls
#import estimate_head_pose as hpe
from face_detector_yolo import getFaces
from picamera import PiCamera
from picamera.array import PiRGBArray
from estimate_head_pose import estimate_direction
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
compte=1

for frame1 in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    time_start = time.perf_counter()
    curr_time = time.perf_counter()
    if i != 0 :
        fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")
        fichier.write(str(curr_time-t0)+" 0\n")
        compte=compte+1
        fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")

    frame = frame1.array

    cv2.imshow('img', frame)

    cv2.waitKey(1)

    #detect faces
    faces = getFaces(frame)

    curr_time = time.perf_counter()
    if i != 0 :
        fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")
        fichier.write(str(curr_time-t0)+" 0\n")
        compte=compte+1
        fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")



    if len(faces) == 0 :

        print("pas de face")

        # Give dir for a human
        dir_people = dp.detect(frame)

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

        curr_time = time.perf_counter()
        if i != 0 :
            fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")
            fichier.write(str(curr_time-t0)+" 0\n")
            compte=compte+1
            fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")

        #print(faces[0][0], faces[0][2], faces[0][1], faces[0][3])
        
        # Trouver le centre du carré
        xMil = int((faces[0][0] + faces[0][2])/2)
        yMil = int((faces[0][1] + faces[0][3])/2)
    
        print(xMil, yMil)
    
        image = cv2.circle(frame, (xMil,yMil), radius=3, color=(0, 0, 255), thickness=2)
        cv2.imshow('img', frame)
        cv2.waitKey(1)
        
        height, width = image.shape[:2]
        
        if xMil < (width/2)-30 :
            print("a gauche !!!")
            
        elif xMil > (width/2)+30:
            print("a droite !!!")
            
        if yMil > height-80 :
            print("trop bas !!")
        elif yMil <  50 :
            print("trop haut !!")
            
        if (width/2)-30 < xMil < (width/2)+30 :
            if  50 < yMil < height-80 :
                # Introduce mark_detector to detect landmarks.
                
                mark_detector = MarkDetector()

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

                    pwm_12.ChangeDutyCycle(0)
                    pwm_32.ChangeDutyCycle(0)

                    frame = frame1.array

                    # Feed frame to image queue.
                    img_queue.put(frame)

                    # Get face from box queue.
                    facebox = box_queue.get()

                    if facebox is not None:
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
                    else:
                      break

                    # Show preview.
                    cv2.imshow("Preview", frame)
                    if cv2.waitKey(10) == 27:
                        break

                    rawCapture.truncate(0)


                # Clean up the multiprocessing process.
                box_process.terminate()
                box_process.join()
                pwm_12.stop()
                pwm_32.stop()
                GPIO.cleanup()

    time_end = time.perf_counter()
    print(time_end-time_start)
    rawCapture.truncate(0)
    i=i+1


    curr_time = time.perf_counter()
    if i != 0 :
        fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")
        fichier.write(str(curr_time-t0)+" 0\n")
        if i == 100 :
            break
        compte=1
        fichier.write(str(curr_time-t0)+" "+str(compte)+"\n")

# Arduino Esclave

    # Récupérer valeur capteur

    # Dire si obstacle ou non


# Arduino Maitre

    # Recuperer direction pour éviter obstacle

    # Recuperer direction pour aller vers l'objectif

    # Se mettre en mouvement vers cette direction

