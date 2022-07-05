# Server

This folder stores the code of the **MoCap server side**.

## Organization

    ├── /old                # codes for other experiments phases. 
    |                       # check the paper to understand the different phases.
    |
    ├── serverv2Calib.py    # runs the real-time extrinsics calibration
    ├── serverv2Test.py     # runs the real-time ground calibration
    ├── serverv2Ground.py   # runs the real-time MoCap capture   
    |
    ├── myLib.py            # function lib
    ├── constants.py        # where you will put the intrisics matrices
    |
    ├── start.sh            # start the ptp server
    |
    ├── requirements.txt    # python requirements
    |
    └── debugOnline.ipynb   # .ipynb to debug the .csv offline



## Requirements

- Install requirements
``` shell 
sudo apt-get install ptpd
pip3 install -r requirements.txt
```

- Start the `ptpd` server
``` bash
source start.sh
```
- Copy the matrices of the intrinsics cali  bration done in [`./calib`](/calib/) to `constants.py` in the format:
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


## Usage

>  **FIRST TIME USERS** : I recommend undestanding `debugOnline.ipynb` before trying to change the real-time python scripts.

You can find the arguments of each script running 
``` python
python3 serverv2Calib.py --help
python3 serverv2Ground.py --help
python3 serverv2Test.py --help
```

Here are some examples that I use per default
``` python
python3 serverv2Calib.py  -n 4 -trig 20 -rec 120 -save -ip 192.168.0.105,192.168.0.100,192.168.0.101,192.168.0.103
python3 serverv2Ground.py -n 4 -trig 2  -rec 10  -save -ip 192.168.0.105,192.168.0.100,192.168.0.101,192.168.0.103
python3 serverv2Test.py   -n 4 -trig 5  -rec 60  -save -ip 192.168.0.105,192.168.0.100,192.168.0.101,192.168.0.103
```
