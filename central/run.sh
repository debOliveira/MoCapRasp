rm -rf ../results/raw/camera1/*
rm -rf ../results/raw/camera2/*
rm -rf ../results/*csv
rm -rf ../results/camera*jpg
python3 server1.py &
python3 server2.py
cd ../results
python3 zipDS.py
cd ../central
