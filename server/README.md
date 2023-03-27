# Server

This folder stores the code of the **MoCap server side**.

## Organization

    ├── requirements.txt    # Python requirements
    |
    ├── mcr                 # Custom MoCap Rasp module package 
    |
    ├── connect.sh          # Start the ptpd server
    |
    ├── calib.py            # Runs the real-time extrinsics calibration
    ├── ground.py           # Runs the real-time ground calibration
    ├── capture.py          # Runs the real-time MoCap capture   
    |
    └── debugOnline.ipynb   # .ipynb to debug the .csv offline



## Requirements

- Install requirements
``` shell 
sudo apt-get install ptpd
sudo apt-get install -y libatlas-base-dev libhdf5-dev libhdf5-serial-dev libjasper-dev
pip3 install -r requirements.txt
```

- Start the `ptpd` server
``` bash
source connect.sh
```
- Copy the matrices of the intrinsics calibration done in [`./calib`](/calib/) to `mcr/constants.py` in the format:
``` python
cameraMatrix_cam1 = np.array([[720.313,0,481.014],
                              [0,719.521,360.991],
                              [0,0,1]])
distCoef_cam1 = np.array([[0.395621],
                          [0.633705],
                          [-2.41723],
                          [2.11079]], dtype=np.float32)
```
and add all matrices to the array `cameraMat` and `distCoef`.

- Make sure mDNS is activated in your network and change the hostname of each Raspberry at the line 
```python
ip = (socket.gethostbyname('cam1.local')+','+socket.gethostbyname('cam2.local')+','+
      socket.gethostbyname('cam3.local')+','+socket.gethostbyname('cam4.local'))
```
at `calib.py`, `ground.py` and `capture.py`. Ensure that the sequence is from camera 1 to N. 

## Usage

>  **FIRST TIME USERS** : I recommend undestanding `debugOnline.ipynb` before trying to change the real-time python scripts.

You can find the arguments of each script running 
``` python
python3 calib.py --help
python3 ground.py --help
python3 capture.py --help
```

Here are some examples that I use per default
``` python
python3 calib.py   -trig 20 -rec 120 -save 
python3 ground.py  -trig 2  -rec 10  -save 
python3 capture.py -marker 4 -trig 5  -rec 60  -save 
```
