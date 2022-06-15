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

# camera matrices
cameraMatrix_cam1 =np.array([[720.313,0,481.014],[0,719.521,360.991],[0,0,1]])
distCoef_cam1 = np.array([[0.395621],[0.633705],[-2.41723],[2.11079]], dtype=np.float32)

cameraMatrix_cam2 =np.array([[768.113,0.,472.596],[0.,767.935,350.978],[0,0,1]])
distCoef_cam2 = np.array([[0.368917],[1.50111],[-7.94126],[11.9171]], dtype=np.float32)

# all in one
cameraMat = [cameraMatrix_cam1,cameraMatrix_cam2]
distCoef = [distCoef_cam1,distCoef_cam2]