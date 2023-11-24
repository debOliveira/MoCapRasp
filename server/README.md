# Server

## 📂 Organization

    ├── requirements.txt    # Python requirements
    |
    ├── setup.py            # Setup script for enviroment
    |
    ├── /mcr/               # MoCap Rasp Module Package 
    |
    ├── connect.sh          # Start the PTPd server
    |
    ├── mocaprasp.py        # CLI Application that runs the real-time Capture Processes
    |
    └── /debug/             # Debug folder with Jupyter Notebook to debug the .csv offline

## 🏗️ Requirements

>  **FIRST-TIME USERS**: follow these steps to setup the application

It's highly recommended to create a Python Virtual Enviroment for `requirements.txt` installation.

- Run the following
``` shell 
sudo apt-get install ptpd
sudo apt-get install -y libatlas-base-dev libhdf5-dev libhdf5-serial-dev libjasper-dev
pip3 install --editable .
```

- Start the `ptpd` server
``` bash
source connect.sh
```
- Copy the matrices of the intrinsics calibration done in [`../calib/`](/calib/) to `./mcr/misc/constants.py` to the array `cameraMat` and `distCoef`.

- Make sure mDNS is activated in your network and change the hostname of each Raspberry at the line to 'camX', where X is a number representing the camera ID. Do not use the same ID for different clients.

## ⚔️ Usage

>  **FIRST-TIME USERS**: understand `../debug/debugOffline.ipynb` before changing the real-time python scripts.

You can find the commands for the system at:
``` python
mocaprasp --help
```

You can find the options of each Capture Process at:
``` python
mocaprasp cec --help
mocaprasp gpe --help
mocaprasp scr --help
```

## 🖼️ Example pics

<p align="center">
<img src="https://user-images.githubusercontent.com/48807586/177630567-203f1129-dc2e-4f36-b9f5-4dd15c5e4c1d.png" height="300" align="center">
<img src="https://user-images.githubusercontent.com/48807586/177630590-ecf808d8-98f1-44f5-a3be-9fbb3b7a658d.png" height="300" align="center"><br><br>
</p>
