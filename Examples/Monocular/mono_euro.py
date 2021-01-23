from ORBSLAM3 import System, eSensor, VectorPoint
import cv2
import os
import time

# example files used for this script:
# http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_05_difficult/MH_05_difficult.zip

video_image_dir = 'mav0/cam0/data'  # data downloaded from above link and extracted to this folder
camera_settings_file = 'EuRoC.yaml'
vocabulary_file = '../../Vocabulary/ORBvoc.txt'  # Don't forget to unzip the vocabulary
show_gui = True

file_names = os.listdir(video_image_dir)
file_names.sort()  # in case they aren't in order
# create list of tuple (file_path, timestamp) <- assumes files are named as timestamp
images = [(os.path.join(video_image_dir, file), float(file[:-4])/1e9) for file in file_names]

# initialize Slam
slam = System(vocabulary_file, camera_settings_file, eSensor.MONOCULAR, True)

for i in range(len(images) - 1):
    img = cv2.imread(images[i][0], 0)  # load image as grayscale
    start = time.time()

    slam.TrackMonocular(img, images[i][1], VectorPoint(), "")

    while time.time() - start < images[i+1][1] - images[i][1]:  # wait until it's time to get next image
        time.sleep(0.00001)

slam.Shutdown()
