## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
## Dev 1
pipeline_1 = rs.pipeline()
config_1 = rs.config()
config_1.enable_device('749512060607')
## Dev 2
pipeline_2 = rs.pipeline()
config_2 = rs.config()
config_2.enable_device('749512060252')

# Get device product line for setting a supporting resolution
config_1.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
config_2.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)

# Start streaming
pipeline_profile = pipeline_1.start(config_1)
k = 0
device = pipeline_profile.get_device()
depth_sensor = device.query_sensors()[0]
set_emitter = 0
depth_sensor.set_option(rs.option.emitter_enabled, set_emitter)
depth_sensor.set_option(rs.option.enable_auto_exposure, False)
depth_sensor.set_option(rs.option.enable_auto_white_balance, False)
depth_sensor.set_option(rs.option.exposure, 15000)
depth_sensor.set_option(rs.option.gain, 90)
pipeline_profile = pipeline_2.start(config_2)
device = pipeline_profile.get_device()
depth_sensor = device.query_sensors()[0]
set_emitter = 0
depth_sensor.set_option(rs.option.emitter_enabled, set_emitter)
depth_sensor.set_option(rs.option.enable_auto_exposure, False)
depth_sensor.set_option(rs.option.enable_auto_white_balance, False)
depth_sensor.set_option(rs.option.exposure, 15000)
depth_sensor.set_option(rs.option.gain, 90)
i = 0

clean1 = cv2.imread('C:/Users/Debora/Desktop/dataset4/cleanSlate1.jpg', cv2.IMREAD_GRAYSCALE)
clean2 = cv2.imread('C:/Users/Debora/Desktop/dataset4/cleanSlate2.jpg', cv2.IMREAD_GRAYSCALE)

try:
    while (k!= 27 and i!=999):

        # Wait for a coherent pair of frames: depth and color
        frames_1 = pipeline_1.wait_for_frames()
        frames_2 = pipeline_2.wait_for_frames()
        color_frame_1 = frames_1.get_infrared_frame()
        color_frame_2 = frames_2.get_infrared_frame()
        if not color_frame_1 or not color_frame_2 :
            continue

        # Convert images to numpy arrays
        img1 = np.asanyarray(color_frame_1.get_data())  
        img2 = np.asanyarray(color_frame_2.get_data())
        images1 = np.hstack((img1, cv2.absdiff(img1,clean1)))
        images1 = np.hstack((images1, cv2.subtract(img1,clean1)))
        images2 = np.hstack((img2, cv2.absdiff(img2,clean2)))
        images2 = np.hstack((images2, cv2.subtract(img2,clean2)))
        images = np.vstack((images1, images2))
        
        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        #cv2.imwrite('C:/Users/Debora/Desktop/dataset4/cleanSlate1.jpg',img1)
        #cv2.imwrite('C:/Users/Debora/Desktop/dataset4/cleanSlate2.jpg',img2)
        cv2.imwrite('C:/Users/Debora/Desktop/dataset4/camera1/'+str(i).zfill(4)+'.jpg',img1)
        cv2.imwrite('C:/Users/Debora/Desktop/dataset4/camera2/'+str(i).zfill(4)+'.jpg',img2) 
        k = cv2.waitKey(1)
        i = i+1

finally:

    # Stop streaming
    pipeline_1.stop()
    pipeline_2.stop()
    cv2.destroyAllWindows()
