import sys
import time 

import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

from sensor_msgs.msg import CompressedImage 

class roomDetector():
    def __init__(self):

        self.detectedRooms = ["Red", "Room2"]
        # Subsriber to get the compressed images from the camera
        self.camera_sub = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, self.find_ball, queue_size=1)


    def find_ball(ros_image):
        #### direct conversion to CV2 ####
        ## @param image_np is the image decompressed and converted in OpendCv
        np_arr = np.fromstring(ros_image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  

        blackLower = (0, 0, 0)
        blackUpper = (5,50,50)
        redLower = (0, 50, 50)
        redUpper = (5, 255, 255)
        yellowLower = (25, 50, 50)
        yellowUpper = (35, 255, 255)
        greenLower = (50, 50, 50)
        greenUpper = (70, 255, 255)
        blueLower = (100, 50, 50)
        blueUpper = (130, 255, 255)
        magentaLower = (125, 50, 50)
        magentaUpper = (150, 255, 255)
