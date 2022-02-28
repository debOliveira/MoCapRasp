from cv2.fisheye import undistortPoints
import numpy as np
from sklearn import linear_model
from constants import distCoef,cameraMatrix,allCombinationsOf3

def myUndistortPointsFisheye(pts,K,D):
    # save variables
    pts,fx,fy,cx,cy = pts.reshape(-1,1,2),K[0,0],K[1,1],K[0,2],K[1,2]
    # remove destortion
    undPts_norm = undistortPoints(pts, K, D)
    undPts_norm = undPts_norm.reshape(-1,2)
    # remove normalization
    undistPts= np.zeros_like(undPts_norm)
    for i, (x, y) in enumerate(undPts_norm):
        px,py = x*fx + cx,y*fy + cy
        undistPts[i,0],undistPts[i,1] = px,py
    return undistPts

def getCoordinate(center,idx,cameraMatrix,distCoef):
    coord,k = np.zeros((len(idx),2)),0    
    for i in idx:
        coord[k] = center[int(i)]  
        k+=1
    undPt = myUndistortPointsFisheye(np.array(coord), cameraMatrix, distCoef)
    return undPt

def isCollinear(p0, p1, p2):
    # coordinated of all centers
    X,y = [[p0[0]],[p1[0]],[p2[0]]],[p0[1], p1[1], p2[1]] 
    reg = linear_model.LinearRegression()       # create LS model
    reg.fit(X, y)                               # fit model
    a3,b3 = reg.coef_,reg.intercept_            # get line coeficients

    y0_LS,y1_LS,y2_LS = abs(int(a3*p0[0] + b3 - p0[1])),abs(int(a3*p1[0] + b3 - p1[1])),abs(int(a3*p2[0] + b3 - p2[1]))     
    # find residual between y and the coordinates Y on the LS line    
    m = (y0_LS + y1_LS + y2_LS)/3               # get mean value of error
    # if it is less than 2 pixel, they are co'llinear
    res = (m < 1)
    return res

def reshapeCoord(coord):
    # first reshape array of coordinates
    coordCopy = np.array(coord).reshape(6)
    # get the Y coordinates of each markers' center
    coordX,coordY = [coordCopy[0], coordCopy[2], coordCopy[4]],[coordCopy[1], coordCopy[3], coordCopy[5]]
    return coordX,coordY

def getOrder(centerX,centerY, baseAxis=False, axis = 1):
    # get wand direction
    distY,distX = np.array(centerY).max()-np.array(centerY).min(),np.array(centerX).max()-np.array(centerX).min() 
    # define the order of the markers 
    if not baseAxis:  #if there is no axis to compare, get maximum dist
        if distY > distX: order,axis = np.argsort(centerY),1
        else: order,axis = np.argsort(centerX),0
    else:  # if there is a previous frame, compare to its axis
        if axis: order = np.argsort(centerY)
        else: order = np.argsort(centerX)
    return order, axis

def processCentroids_test(coord,a0,b0):
    # INIT VARIABLES
    collinearCentroids,onePassFlag,fourRLinearFlag = [],False,False
    diameter,centerCoord,doubleCollinearIdx,meanDist = [0,0,0,0],np.zeros((3,2)),np.zeros((2,3)),np.zeros((4,1)) 
    
    # get collinear blobs
    for i in allCombinationsOf3:
        centerCoord = getCoordinate(coord,i,cameraMatrix,distCoef)  
        if isCollinear(*centerCoord):
            if onePassFlag:
                ourRLinearFlag,doubleCollinearIdx[0],doubleCollinearIdx[1] = True,collinearCentroids,i
                continue
            collinearCentroids,onePassFlag = i,True    
    # if there is ambiguity
    if fourRLinearFlag: 
        k=0
        for i in allCombinationsOf3:
            centerCoord = getCoordinate(coord,i,cameraMatrix,distCoef)  
            center = np.mean(centerCoord,axis=0)
            meanDist[k] = np.mean(np.linalg.norm(centerCoord-center,axis=1))
            k+=1
        if allCombinationsOf3[np.argmin(meanDist)] in doubleCollinearIdx:
            collinearCentroids = allCombinationsOf3[np.argmin(meanDist)]
    # get fourth marker index
    is0, = np.where(collinearCentroids == 0)
    is1, = np.where(collinearCentroids == 1)
    is2, = np.where(collinearCentroids == 2)
    if len(is0):
        if len(is1):
            if len(is2): markerDPosition = 3
            else: markerDPosition = 2
        else: markerDPosition = 1
    else: markerDPosition = 0
    # order markers
    idx = np.hstack((collinearCentroids,markerDPosition))
    coord = coord[idx]  
    centerCoord = np.zeros((4,2)) 
    for i in range(0,4):
        centerCoord[i] = [coord[i][0]+b0-5,coord[i][1]+a0-5] 
    return centerCoord