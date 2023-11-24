import time
from os import path
import numpy as np

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Init client 
client = RemoteAPIClient() # Client object 
client.setStepping(True)   # Client will manually call the next simulation step

sim = client.getObject('sim') # Simulation object
fps = 60 # Frames per second
simTimeStep = 1/fps

quadcopterHandle = sim.getObject('/Quadcopter')
trackingPath = path.join(path.dirname(path.abspath(__file__)), 'tracking.csv')
tracking = np.genfromtxt(trackingPath, delimiter=',')

markersPath = path.join(path.dirname(path.abspath(__file__)), 'markers.csv')
markers = np.genfromtxt(markersPath, delimiter=',')

camerasPath = path.join(path.dirname(path.abspath(__file__)), 'cameras.csv')
cameras = np.genfromtxt(camerasPath, delimiter=',')

markersPointCloud = sim.createPointCloud(1, 1, 8, 3)
markersColor = list([197,29,52]) # Raspberry red. Get it?

sim.startSimulation()

# Simulate camera system
for ID, cameraPose in enumerate(cameras):
    cameraHandle = sim.getObject(f'/Camera[{ID}]')
    sim.setObjectMatrix(cameraHandle, -1, cameraPose.tolist()) # Put camera to its respective pose
    sim.setObjectOrientation(cameraHandle, list([0,0,np.pi]), cameraHandle) # Rotate camera object to match Coppelia's image display 
    sim.setModelProperty(cameraHandle, sim.getModelProperty(cameraHandle) &~ sim.modelproperty_not_visible) # Make camera visible

# Show marker animation
for points in markers:
    # Insert points into point cloud
    totalPointCnt = sim.insertPointsIntoPointCloud(markersPointCloud, 0, points, markersColor)
    time.sleep(simTimeStep)
    client.step()
    # Remove all points from point cloud. This will refresh each frame
    totalPointCnt = sim.removePointsFromPointCloud(markersPointCloud, 0, None, 0)
sim.removeObjects([markersPointCloud]) # Delete point cloud object

# Show quadcopter pose tracking
sim.setModelProperty(quadcopterHandle, sim.getModelProperty(quadcopterHandle) &~ sim.modelproperty_not_visible) # Make Quadcopter visible
for pose in tracking:
    sim.setObjectMatrix(quadcopterHandle, -1, pose.tolist()) # Set quadcopter pose
    time.sleep(simTimeStep)
    client.step()
sim.setModelProperty(quadcopterHandle, sim.getModelProperty(quadcopterHandle) | sim.modelproperty_not_visible) # Make Quadcopter invisible again

# Disable camera visibility
for ID, cameraPose in enumerate(cameras):
    cameraHandle = sim.getObject(f'/Camera[{ID}]')
    sim.setModelProperty(cameraHandle, sim.getModelProperty(cameraHandle) | sim.modelproperty_not_visible) # Make camera invisible

sim.stopSimulation()