from ORBSLAM3 import System, eSensor, VectorPoint
import cv2
import time

cap = cv2.VideoCapture(1)  # stream iphone to computer using epoccam

camera_settings_file = 'iphone.yaml'
vocabulary_file = '../../Vocabulary/ORBvoc.txt'  # Don't forget to unzip the vocabulary
show_gui = True

# initialize Slam
slam = System(vocabulary_file, camera_settings_file, eSensor.MONOCULAR, True)
input("Press enter to start slam...")

while True:
    try:
        ret, img = cap.read()

        slam.TrackMonocular(img, time.time(), VectorPoint(), "")
    except KeyboardInterrupt as e:
        break

slam.Shutdown()
