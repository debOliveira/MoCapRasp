# Intrinsics calibration

This folder stores the code for the **collection of the data** used in the **calibration of the focal distance, principal point and distortion parameters**. 

## Requirements

- Make sure you [installed the `libcamera`](https://www.raspberrypi.com/documentation/accessories/camera.html#:~:text=Run%20sudo%20raspi%2Dconfig%20.,Reboot%20your%20Raspberry%20Pi%20again) using `raspi-config` :arrow_right: `interface`.
- Check if the `picamera` lib is installed (it should be by default at the Raspbian distro).
``` python
python3
import picamera
```
- Create a folder named `pics/` where you are running the code.
``` bash
mkdir pics
```
- Have a bright light source directly behind the camera.
    * I use a ring light
- Print a [calibration pattern](https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html).
    * I used a 11 X 12 with 30cm side square

## Setting parameters

### Resolution and sensor mode

Change the values of the resolution of the image and the [sensor mode](https://picamera.readthedocs.io/en/release-1.13/fov.html#sensor-modes) at 
```python
picamera.PiCamera(resolution=(640,480), framerate=20, sensor_mode=2)
```
- Prefer bigger resolutions and modes with full sensor capture
- In the motion capture, you can use any resolution or sensor size below the value you setted at the intrinsic calibration

### Framerate

Change the FPS to the lower limit of the sensor mode you choose. Check the [table](https://picamera.readthedocs.io/en/release-1.13/fov.html#sensor-modes) here.

- **If you are saving pitch black images, increasing the FPS a little may solve your issue**.

### Gain

The image gain is set to adjust automaticaly at the first 5 seconds of the code. After that, it locks.

- **I usually approximate the calibration pattern in these 5 seconds so the image very bright**.

## Usage

- Run the line `python3 calibCapture.py`
- Hit <kbd>Ctrl+C</kbd> when you want the capture to stop.
- Copy the images to the server (I used `scp`) and run the [camera calibrator application](https://github.com/debOliveira/myCameraCalibrator).

