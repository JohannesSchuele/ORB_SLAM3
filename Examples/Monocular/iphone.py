from ORBSLAM3 import System, eSensor, VectorPoint, Point
import json
import cv2
import time

# ********************************************************************************************************
# ************************  USER SETTINGS ****************************************************************
cap = cv2.VideoCapture(1)  # stream iphone to computer using epoccam
camera_settings_file = 'iphone.yaml'
vocabulary_file = '../../Vocabulary/ORBvoc.txt'  # Don't forget to unzip the vocabulary
use_imu_data = True  # set true if using SensorLog app on phone (use phone as server)
imu_ip = "192.168.137.153"  # update to match SensorLog
imu_port = 7777  # update to match SensorLog
show_gui = True
# ********************************************************************************************************
# ********************************************************************************************************
imu_data = VectorPoint()
is_running = [False]

if use_imu_data:
    from DataSocket import TCPReceiveSocket  # pip install PyDataSocket
    g = 9.81


    def receive_data(data):
        if is_running[0]:
            data_string = data.decode("utf-8")
            lines = data_string.split('\n')
            for line in lines:
                try:
                    data = json.loads(line)
                    imu_data.append(Point(float(data['accelerometerAccelerationX']) * g,
                                          float(data['accelerometerAccelerationY']) * g,
                                          float(data['accelerometerAccelerationZ']) * g,
                                          float(data['gyroRotationX']),
                                          float(data['gyroRotationY']),
                                          float(data['gyroRotationZ']),
                                          time.time()))
                except Exception as e:  # <- bad practice
                    pass


    socket = TCPReceiveSocket(tcp_ip=imu_ip, tcp_port=imu_port, handler_function=receive_data, receive_as_raw=True, as_server=False)
    socket.start()

# initialize Slam
slam = System(vocabulary_file, camera_settings_file, eSensor.MONOCULAR, True)
input("Press enter to start slam...")
is_running[0] = True

while True:
    try:
        ret, img = cap.read()
        start = time.time()
        slam.TrackMonocular(img, time.time(), imu_data, "")
        imu_data.clear()
        while time.time() - start < 1/20:
            time.sleep(0.001)

    except KeyboardInterrupt as e:
        break

slam.Shutdown()
