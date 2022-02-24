import warnings
warnings.filterwarnings("ignore")
import glob
import time
import os,shutil
import numpy as np
import argparse
from PIL import Image
import cv2
from itertools import combinations
from functions import myUndistortPointsFisheye,getCoordinate,isCollinear,reshapeCoord,getOrder

params = cv2.SimpleBlobDetector_Params()  
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
detector = cv2.SimpleBlobDetector_create(params)
diameter = [0,0,0,0]
allIdx = list(range(0,4))
allCombinationsOf3 = np.array(list(combinations(allIdx,3)))
centerCoord = np.zeros((3,2)) 
doubleCollinearIdx = np.zeros((2,3)) 
meanDist = np.zeros((4,1)) 
cameraMatrix =np.array([[816.188,0,318.382],
                                [0,814.325,250.263],
                                [0,0,1]])
distCoef = np.array([[-0.292355],[0.199853],[0.386838],[-6.51433]], dtype=np.float32)
k = 0

# clean results folder   
def cleanFolders(number):
    try:
        if os.path.isdir('../results/raw'): 
            if glob.glob('../results/results_'+str(number)+".csv"):
                os.system('rm -rf ../results/results_'+str(number)+".csv ../results/*zip")
            shutil.rmtree('../results/raw/camera'+str(number))
            os.makedirs('../results/raw/camera'+str(number))
    except OSError as e:
        print("Error erasing folder", e.strerror)

# argument parser configuration
msg = "This function checks for new images in the 'results/raw/camera' folder"
parser = argparse.ArgumentParser(description = msg)
parser.add_argument("-c", "--camera", help = "Number X of the camera that should be listened")
args = parser.parse_args()

# init variables
number = int(args.camera)
cleanFolders(number)
newFiles = []
myFiles = []
locked = True

# wait if there are no files in folder
print('[INFO] waiting recording from camera '+str(number))
while not len(newFiles): 
    time.sleep(0.001)
    newFiles = glob.glob("../results/raw/camera"+str(number)+"/*jpg")

# init capture from files until csv is trasnfered
print('[INFO] capture started')
while not len(glob.glob("".join(["../results/results_",str(number),".csv"]))):
    # get difference between old files and actual files in folder
    newFiles = np.setdiff1d(glob.glob("".join(["../results/raw/camera",str(number),"/*jpg"])), myFiles)
    # concatenate old files with the new file
    myFiles = np.concatenate((myFiles,newFiles))
    if len(newFiles):
        print(newFiles)
        for imgName in newFiles:
            collinearCentroids = []
            onePassFlag = False
            fourRLinearFlag = False
            while locked:
                try:
                    im = np.array(Image.open(imgName))
                    if len(im): locked = False
                except: continue
            locked = True
            nonNullCoord = im > 127
            if np.any(nonNullCoord):
                a,b = np.nonzero(nonNullCoord.argmax(1))[0], np.nonzero(nonNullCoord.argmax(0))[0]
                imgMasked = cv2.bitwise_not(im[a[0]-5:a[-1]+5,b[0]-5:b[-1]+5])
                keypoints = detector.detect(imgMasked)
                N = np.array(keypoints).shape[0] 
                if N > 4:
                    for i in range(0,N): diameter[i] = (keypoints[i].size)
                    orderAscDiameters = np.argsort(diameter)
                    keypoints = [keypoints[orderAscDiameters[0]],keypoints[orderAscDiameters[1]],keypoints[orderAscDiameters[2]]]
                elif N == 4:
                    for i in allCombinationsOf3:
                        centerCoord = getCoordinate(keypoints,i,cameraMatrix,distCoef)  
                        if isCollinear(*centerCoord):
                            if onePassFlag:
                                fourRLinearFlag,doubleCollinearIdx[0],doubleCollinearIdx[1] = True,collinearCentroids,i
                                continue
                            collinearCentroids,onePassFlag = i,True    
                    if fourRLinearFlag: 
                        for i in allCombinationsOf3:
                            centerCoord = getCoordinate(keypoints,i,cameraMatrix,distCoef)  
                            center = np.mean(centerCoord,axis=0)
                            meanDist[k] = np.mean(np.linalg.norm(centerCoord-center,axis=1))
                            k+=1
                        if allCombinationsOf3[np.argmin(meanDist)] in doubleCollinearIdx:
                            collinearCentroids = allCombinationsOf3[np.argmin(meanDist)]
                    is0, = np.where(collinearCentroids == 0)
                    is1, = np.where(collinearCentroids == 1)
                    is2, = np.where(collinearCentroids == 2)
                    if len(is0):
                        if len(is1):
                            if len(is2):
                                markerDPosition = 3
                            else:
                                markerDPosition = 2
                        else:
                            markerDPosition = 1
                    else:
                        markerDPosition = 0
                    idx = np.hstack((collinearCentroids,markerDPosition))
                    keypoints = np.array(keypoints)[idx]  
                    centerCoord = np.zeros((4,2)) 
                    for i in range(0,4):
                        centerCoord[i] = [keypoints[i].pt[0]+b[0]-5,keypoints[i].pt[1]+a[0]-5]  
                    centerCoord = myUndistortPointsFisheye(np.array(centerCoord), cameraMatrix, distCoef)
                        

print('[FINISHED]')
