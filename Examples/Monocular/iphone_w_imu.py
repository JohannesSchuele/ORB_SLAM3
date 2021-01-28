from ORBSLAM3 import System, eSensor, VectorPoint, Point
from DataSocket import TCPReceiveSocket
import json
import cv2
import time


imu_data = VectorPoint()
g = 9.81
is_running = [False]


def print_data(data):
    try:
        data = json.loads(data.decode("utf-8"))
    except:
        return
    if is_running[0]:
        imu_data.append(Point(data['accelerometerAccelerationX']*g,
                         data['accelerometerAccelerationY']*g,
                         data['accelerometerAccelerationZ']*g,
                         data['gyroRotationX'],
                         data['gyroRotationY'],
                         data['gyroRotationZ'],
                         time.time()))
        

socket = TCPReceiveSocket(tcp_ip="192.168.137.109", tcp_port=7777, handler_function=print_data, receive_as_raw=True, as_server=False)
socket.start()

cap = cv2.VideoCapture(1)  # stream iphone to computer using epoccam

camera_settings_file = 'iphone.yaml'
vocabulary_file = '../../Vocabulary/ORBvoc.txt'  # Don't forget to unzip the vocabulary
show_gui = True

# initialize Slam
slam = System(vocabulary_file, camera_settings_file, eSensor.MONOCULAR, True)
input("Press enter to start slam...")
is_running[0] = True

while True:
    try:
        ret, img = cap.read()
        slam.TrackMonocular(img, time.time(), imu_data, "")
        imu_data.clear()
    except KeyboardInterrupt as e:
        break

slam.Shutdown()
