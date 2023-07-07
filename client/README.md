# Client

This folder stores the code of the **MoCap client side**. We used RaspPi 4b 16GB RAM with Raspbian 32 bits. 

## Organization

    ├── LED.py              # turns on IR ring if connected to GPIO
    ├── record.py           # record the images thread
    ├── watch.py            # processing image thread  
    |
    ├── run.sh              # bash to run capture
    ├── test.sh             # bash to test the camera image
    ├── connect.sh          # start the ptp server
    |
    ├── requirements.txt    # python requirements
    |
    └── raspividyuv         # modified command to call camera wrapper


## Requirements

- When installing the [Raspbian OS](https://www.raspbian.org/), change the hostname to `camX`, for `X` the number of the camera

- Access the Raspberry remotely (e.g. for camera 1)
``` shell 
ssh pi@cam1.local
``` 

- Install requirements
``` shell 
sudo apt-get install ptpd
sudo apt-get install -y libatlas-base-dev libhdf5-dev libhdf5-serial-dev libjasper-dev
pip3 install -r requirements.txt
```

- Deactivate NTP server
``` bash
systemctl disable systemd-timesyncd.service
```

- Start the `ptpd` server
``` bash
source connect.sh
```
- Connect a jumper to the GPIO 4 if you want the RPi to trigger the LED
- Test if the camera stream can be opened
``` bash
source test.sh
```
- Check the image using **VNC server** 
>    * Activate VNC in `sudo raspi-config` > Interface
>    * Install **VNC viewer** in your Desktop and add a new connection with your Rasp IP
>    * Uncomment `hdmi-hot-plug` in `boot/config.txt` if using a Rasp 4
>    * Activate direct capture under **VNC server** > Options
>    * If you receive a warning to unsupported privilege, run `sudo vncpasswd -service`
- Adjust the blob detectors parameters in `watch.py` to the size of your blob
>    1) Change properties of `params` in `watch.py`
>    1) Comment the [lines 45 to 54](https://github.com/debOliveira/MoCapRasp/blob/main/client/record.py#L45-L54) in `record.py`and uncomment [line 55](https://github.com/debOliveira/MoCapRasp/blob/main/client/record.py#L55)
>    2) Uncomment the [lines 61 to 67](https://github.com/debOliveira/MoCapRasp/blob/main/client/record.py#L55) in `watch.py`
>    3) Run `source run.sh` to view the captured blob and centroids
>    4) Repeat steps 1 to 3 until all blobs are detected


## Usage

1) Run the `calib.py`, `ground.py` or `capture.py` in central
2) Source run code
``` bash
source run.sh
```
3) Press <kbd>Ctrl+C</kbd> when capture is finished to avoid waiting for the timeout
> You can change the timeout waiting in line 85 of `watch.py` (default: 300 seconds)

## Turn off

Run the line
```
sudo shutdown -h now
```
and disconnect all power plugs.

