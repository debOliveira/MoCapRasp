# Simulations 

The `virtualArena.ttt` is a CoppeliaSim scene which acts as a digital twin of the arena. It gathers information processed from the debug tool `debugOffline.ipynb` via the produced .csv files in this folder.

For the simulation:
- `cameras.csv` contains the camera's transformation matrix information;
- `markers.csv` contains the triangulated markers coordinates for each valid frame;
- `tracking.csv` contains an experimental algorithm for rigid body tracking of a quadcopter.

To start the simulation, run the `virtualArena.py` script while the scene is open in CoppeliaSim. This Python script communicates with the scene via the CoppeliaSim remote API. 

Make sure to set the correct Python interpreter and has the following API modules installed:
- `python -m pip install cbor`
- `python -m pip install zmq`
- `python -m pip install cbor2`
- `python -m pip install pyzmq`

OBS: *Don't save the scene when closing it to maintain it's original configuration.* 