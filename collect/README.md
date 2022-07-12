# Client

This folder stores the code of the **MoCap server side**.

## Organization

    ├── /old                # codes for other experiments phases. 
    |                       # check the paper to understand the different phases.
    |
    ├── LED.py              # turns on IR ring if connected to GPIO
    ├── capture.py          # captures the images thread
    ├── watch.py            # processing image thread  
    |
    ├── run.sh              # bash to run capture
    ├── test.sh             # bash to test the camera image
    ├── start.sh            # start the ptp server
    |
    ├── requirements.txt    # python requirements
    |
    └── raspividyuv         # .ipynb to debug the .csv offline



## Requirements

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
source start.sh
```
- Connect a jumper to the GPIO 4 if you want the RPi to trigger the LED
- Test if the camera stream can be opened
``` bash
source test.sh
```
- Check the image using **VNC server** 
>    * Activate VNC in `sudo raspi-config` > Interface
>    * Install **VNC viewer** in your Dektop and add a new connection with your Rasp IP
>    * Uncomment `hdmi-hot-plug` in `boot/config.txt` if using a Rasp 4
>    * Activate direct capture under **VNC server** > Options
>    * If you receive a warning to unsupported privilege, run `sudo vncpasswd -service`
- Adjust the blob detectors parameters in `watch.py` to the size of your blob
>    1) Change propertires of `params` in `watch.py`
>    1) Comment the lines 44 to 53 in `capture.py`and uncomment line 54
>    2) Uncomment the lines 59 to 66 in `watch.py`
>    3) Run `source run.sh` to view the captured blob and centroids
>    4) Repeat steps 1 to 3 until all blobs are detected


## Usage

1) Run the `serverv2Calib.py`, `serverv2Calib.py` or `serverv2Calib.py`
2) Source run code
``` bash
source run.sh
```
3) Press <kbd>Ctrl+C</kbd> when capture is finished to avoid waiting for the timeout
> You can change the timeout waiting in line 85 of `watch.py` (default: 300 seconds)
