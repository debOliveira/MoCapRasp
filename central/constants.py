from cv2 import SimpleBlobDetector_Params,SimpleBlobDetector_create
from itertools import combinations
import numpy as np

params = SimpleBlobDetector_Params()  
params.minThreshold = 0    
params.thresholdStep = 100
params.maxThreshold = 200
params.filterByArea = True
params.minArea = 20
params.filterByConvexity = True
params.minConvexity = 0.95
params.minDistBetweenBlobs = 10
params.filterByColor = True
params.blobColor = 0
params.minRepeatability =  1

detector = SimpleBlobDetector_create(params)
allIdx = list(range(0,4))
allCombinationsOf3 = np.array(list(combinations(allIdx,3)))
cameraMatrix =np.array([[816.188,0,318.382],
                                [0,814.325,250.263],
                                [0,0,1]])
distCoef = np.array([[-0.292355],[0.199853],[0.386838],[-6.51433]], dtype=np.float32)