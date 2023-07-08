# Server

## 📂 Organization

    ├── requirements.txt    # python requirements
    |
    ├── mcr                 # MoCapRasp module package 
    |
    ├── connect.sh          # start the ptpd server
    |
    ├── calib.py            # runs the real-time extrinsics calibration
    ├── ground.py           # runs the real-time ground calibration
    ├── capture.py          # runs the real-time capture   
    |
    └── debugOnline.ipynb   # .ipynb to debug the .csv offline



## 🏗️ Requirements

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
- Copy the matrices of the intrinsics calibration done in [`./calib`](/calib/) to `mcr/constants.py` to the array `cameraMat` and `distCoef`.

- Make sure mDNS is activated in your network and change the hostname of each Raspberry at the line 
```python
ip = (socket.gethostbyname('cam1.local')+','+socket.gethostbyname('cam2.local')+','+
      socket.gethostbyname('cam3.local')+','+socket.gethostbyname('cam4.local'))
```
at `calib.py`, `ground.py` and `capture.py`. Ensure that the sequence is from camera 1 to N. 

## ⚔️ Usage

>  **FIRST-TIME USERS**: understand `debugOnline.ipynb` before changing the real-time python scripts.

You can find the arguments of each script running 
``` python
python3 calib.py --help
python3 ground.py --help
python3 capture.py --help
```

Per default: 
``` python
python3 calib.py -trig 20 -rec 120 -save 
python3 ground.py -trig 2 -rec 10 -save 
python3 capture.py -marker 4 -rec 60 -save -trig 5  
```

## 🖼️ Example pics

<p align="center">
<img src="https://user-images.githubusercontent.com/48807586/177630567-203f1129-dc2e-4f36-b9f5-4dd15c5e4c1d.png" height="300" align="center">
<img src="https://user-images.githubusercontent.com/48807586/177630590-ecf808d8-98f1-44f5-a3be-9fbb3b7a658d.png" height="300" align="center"><br><br>
</p>
