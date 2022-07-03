## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.infrared, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline_profile = pipeline.start(config)
device = pipeline_profile.get_device()

depth_sensor = device.query_sensors()[0]
emitter = depth_sensor.get_option(rs.option.emitter_enabled)
print("emitter = ", emitter)
set_emitter = 0 
depth_sensor.set_option(rs.option.emitter_enabled, set_emitter)
emitter1 = depth_sensor.get_option(rs.option.emitter_enabled)
print("new emitter = ", emitter1)
depth_sensor.set_option(rs.option.enable_auto_exposure, False)
depth_sensor.set_option(rs.option.exposure, 5000)
#depth_sensor.set_option(rs.option.gain,16)
#depth_sensor.set_option(rs.option.enable_auto_white_balance, False)

depth_sensor = device.query_sensors()[1]
depth_sensor.set_option(rs.option.enable_auto_exposure, False)

k = 0
try:
    while k != 27:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_infrared_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
             
        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)
        k = cv2.waitKey(1)

finally:

    # Stop streaming
    pipeline.stop()
