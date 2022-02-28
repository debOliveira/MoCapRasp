from cv2.fisheye import undistortPoints
import numpy as np
from sklearn import linear_model
import os,shutil,glob

def myUndistortPointsFisheye(pts,K,D):
    # save variables
    pts = pts.reshape(-1,1,2)
    fx = K[0,0]
    fy = K[1,1]
    cx = K[0,2]
    cy = K[1,2]
    # remove destortion
    undPts_norm = undistortPoints(pts, K, D)
    undPts_norm = undPts_norm.reshape(-1,2)
    # remove normalization
    undistPts= np.zeros_like(undPts_norm)
    for i, (x, y) in enumerate(undPts_norm):
        px = x*fx + cx
        py = y*fy + cy
        undistPts[i,0] = px
        undistPts[i,1] = py    
    return undistPts

def getCoordinate(keypoints,idx,cameraMatrix,distCoef):
    try: 
        coord = np.zeros((len(idx),2))        
        k = 0
        for i in idx:
            selectedKeypoints = keypoints[int(i)]
            coord[k] = [selectedKeypoints.pt[0],selectedKeypoints.pt[1]]  
            k+=1
    except: 
        selectedKeypoints = keypoints[idx]
        coord = [selectedKeypoints.pt[0],selectedKeypoints.pt[1]]  
    undPt = myUndistortPointsFisheye(np.array(coord), cameraMatrix, distCoef)

    return undPt

def isCollinear(p0, p1, p2):
    X = [[p0[0]], [p1[0]], [p2[0]]]             # X coordinated of all centers
    y = [p0[1], p1[1], p2[1]]                   # Y coordinated of all centers
    
    reg = linear_model.LinearRegression()       # create LS model
    reg.fit(X, y)                               # fit model
    
    a3 = reg.coef_                              # get line coeficient A
    b3 = reg.intercept_                         # get line coeficient B

    y0_LS = abs(int(a3*p0[0] + b3 - p0[1]))     # find residual between y and 
    y1_LS = abs(int(a3*p1[0] + b3 - p1[1]))     # the coordinates Y on the LS line
    y2_LS = abs(int(a3*p2[0] + b3 - p2[1]))
    
    m = (y0_LS + y1_LS + y2_LS)/3               # get mean value of error

    # if it is less than 2 pixel, they are co'llinear
    res = (m < 1)

    return res

def reshapeCoord(coord):
    # first reshape array of coordinates
    coordCopy = np.array(coord).reshape(6)
    # get the Y coordinates of each markers' center
    coordX = [coordCopy[0], coordCopy[2], coordCopy[4]]
    coordY = [coordCopy[1], coordCopy[3], coordCopy[5]]
    return coordX,coordY

def getOrder(centerX,centerY, baseAxis=False, axis = 1):
    # get wand direction
    distY = np.array(centerY).max() - np.array(centerY).min()
    distX = np.array(centerX).max() - np.array(centerX).min()   
    # define the order of the markers 
    if not baseAxis:  #if there is no axis to compare, get maximum dist
        if distY > distX:
            order = np.argsort(centerY)
            axis = 1
        else:
            order = np.argsort(centerX)
            axis = 0
    else:  # if there is a previous frame, compare to its axis
        if axis:
            order = np.argsort(centerY)
        else:
            order = np.argsort(centerX)

    return order, axis

def cleanFolders(number):
# clean results folder  
    try:
        if os.path.isdir('../results/raw'): 
            if glob.glob('../results/results_'+str(number)+".csv"):
                os.system('rm -rf ../results/results_'+str(number)+".csv ../results/*zip")
            shutil.rmtree('../results/raw/camera'+str(number))
            os.makedirs('../results/raw/camera'+str(number))
    except OSError as e:
        print("Error erasing folder", e.strerror)
