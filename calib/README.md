# Intrinsics calibration

## 🏗️ Requirements

- Activate [`libcamera`](https://www.raspberrypi.com/documentation/accessories/camera.html#:~:text=Run%20sudo%20raspi%2Dconfig%20.,Reboot%20your%20Raspberry%20Pi%20again) via `sudo raspi-config` :arrow_right: `interface`
- Check `picamera` library (default at the Raspbian distro)
``` python
python3
import picamera
```
- Create `pics/` folder
``` bash
mkdir pics
```
- Have a bright light source directly behind the camera
- Print a [calibration pattern](https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html) - our has 11 X 12 with 30cm side square ([PDF](https://github.com/debOliveira/myCameraCalibrator/tree/main/python/pdf))

## 🧰 Configuration

### Resolution and sensor mode

```python
picamera.PiCamera(resolution=(960,640), framerate=20, sensor_mode=2) # line 7 in calibCapture.py
```
- Refer to [sensor mode docs](https://picamera.readthedocs.io/en/release-1.13/fov.html#sensor-modes)
- Prefer bigger resolutions and modes with full sensor capture
- Use PNG over compressed types (JPG, bitmap)

### Framerate

- Change the FPS to the lower limit of the sensor mode you choose (see the [table]([url](https://picamera.readthedocs.io/en/release-1.13/fov.html#sensor-modes)) 
- If you receive pitch-black images, increase the FPS

### Gain

- The gain is set to adjust automatically at the first 5 seconds
- Approximate the calibration pattern in these 5 seconds so the image is very bright

## ⚔️ Usage

- Run `python3 calibCapture.py`
- Hit <kbd>Ctrl+C</kbd> to stop
- Copy the images to the server (use `scp`) and run the [camera calibrator application](https://github.com/debOliveira/myCameraCalibrator)

## 🖼️ Example pics

 <p align="center">
<img src="https://user-images.githubusercontent.com/48807586/177628962-0bc55667-9e42-4df5-b561-c19c39566cfd.png" height="300" align="center">
<img src="https://user-images.githubusercontent.com/48807586/177629909-fe780ae5-4fff-4817-a7e8-bd28ac0e00a5.png" height="300" align="center"><br><br>
<img src="https://user-images.githubusercontent.com/48807586/177629924-51aee180-2a6a-44b6-892f-6906a22174c9.png" height="300" align="center">
<img src="https://user-images.githubusercontent.com/48807586/177630302-cd32b6c9-6b18-49d8-9d33-7db810586d98.png" height="300" align="center">
</p>
