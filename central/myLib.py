import numpy as np
from sklearn import linear_model
from cv2.fisheye import undistortPoints

def isCollinear(p0, p1, p2):
    X,y = [[p0[0]],[p1[0]],[p2[0]]],[p0[1], p1[1], p2[1]]      
    reg = linear_model.LinearRegression()       # create LS model
    reg.fit(X, y)                               # fit model    
    a3,b3 = reg.coef_,reg.intercept_   
    y0_LS,y1_LS,y2_LS = abs(int(a3*p0[0] + b3 - p0[1])),abs(int(a3*p1[0] + b3 - p1[1])),abs(int(a3*p2[0] + b3 - p2[1]))     
    m = (y0_LS + y1_LS + y2_LS)/3
    res = (m < 1)
    return res

def isEqual(pt,tol=5):
    A,B,C = pt[0],pt[1],pt[2]
    AB,AC,BC = np.linalg.norm(A-B),np.linalg.norm(A-C),np.linalg.norm(C-B)
    return min(AB,AC,BC)<tol

def getPreviousCentroid(noPrevious, lastCentroid):
    if not noPrevious: return []
    else: return lastCentroid

def reshapeCoord(coord):
    # first reshape array of coordinates
    coordCopy = np.array(coord).reshape(6)
    # get the Y coordinates of each markers' center
    coordX,coordY = [coordCopy[0], coordCopy[2], coordCopy[4]],[coordCopy[1], coordCopy[3], coordCopy[5]]
    return coordX,coordY

def getOrder(centerX,centerY, baseAxis=False, axis = 1):
    # get wand direction
    distY,distX = np.array(centerY).max() - np.array(centerY).min(),np.array(centerX).max() - np.array(centerX).min()  
    # define the order of the markers 
    if not baseAxis:  #if there is no axis to compare, get maximum dist
        if distY > distX: order,axis = np.argsort(centerY),1
        else: order,axis = np.argsort(centerX),0
    else:  # if there is a previous frame, compare to its axis
        if axis: order = np.argsort(centerY)
        else: order = np.argsort(centerX)
    return order, axis


def getSignal(num1, num2, tol=10**(-6)):
    if abs(num1-num2) <= tol: return 0,False
    if (num1-num2) < 0:  return -1,True
    else: return 1,True

def swapElements(arr, idx1, idx2):
    aux = arr[idx1]
    arr[idx1] = arr[idx2]
    arr[idx2] = aux
    return arr

def findNearestC(nearestA, nearestB): # get the numer missing from the array [0,1,2]
    vec = np.array([nearestA, nearestB])
    is0, = np.where(vec == 0)
    is1, = np.where(vec == 1)
    is0,is1 = len(is0), len(is1)
    if is0:
        if is1: return 2
        else: return 1
    else: return 0

def orderCenterCoord(centerCoord, prevCenterCoord, otherCamOrder = 0):
    centerX, centerY = reshapeCoord(centerCoord)
    # if it is the first image of the sequence
    if len(prevCenterCoord) == 0:  
        order,_ =  getOrder(centerX,centerY)  
        # if it is the second camera
        if otherCamOrder != 0:  
            # if the markers are wrong, swap the extremities
            signal, valid = getSignal(centerX[order[0]], centerX[order[2]],5)
            if signal != otherCamOrder and valid: order = swapElements(order, 0, 2)    
        else:        
            # get base for comparision (first camera only)        
            otherCamOrder,_ = getSignal(centerX[order[0]], centerX[order[2]])
        # sort centers        
        if np.linalg.norm(centerX[order[0]]-centerX[order[1]])>np.linalg.norm(centerX[order[2]]-centerX[order[1]]):
            sortedCenterCoord = np.array((centerCoord[order[0]], centerCoord[order[1]], centerCoord[order[2]]))
        else: sortedCenterCoord = np.array((centerCoord[order[2]], centerCoord[order[1]], centerCoord[order[0]]))
    else:
        # first reshape array of coordinates
        prevCenterX,prevCenterY = reshapeCoord(prevCenterCoord)
        # distance from marker A/B of previous img to center coordiantes of actual img
        distA = np.sqrt(np.power(np.subtract(prevCenterX[0], centerX), 2) + np.power(np.subtract(prevCenterY[0], centerY), 2))
        distB = np.sqrt(np.power(np.subtract(prevCenterX[1], centerX), 2) + np.power(np.subtract(prevCenterY[1], centerY), 2))
        # nearest marker from A is selected and removed as marker B candidate
        nearestA = np.argmin(distA)
        distBCopy = np.delete(distB, nearestA)
        # nearest marker from B is selected and removed as marker C candidate
        nearestBCopy = np.argmin(distBCopy)
        nearestB, = np.where(distB == distBCopy[nearestBCopy])
        distBCopy = np.delete(distBCopy, nearestBCopy)
        # get the missing marker position in array
        nearestC = findNearestC(nearestA, nearestB[0])
        # sort centers        
        sortedCenterCoord = [centerCoord[nearestA], centerCoord[nearestB[0]], centerCoord[nearestC]]
        # check if the ordering is ok
        centerX, centerY = reshapeCoord(sortedCenterCoord)
        prevOrder,axisPrev = getOrder(prevCenterX,prevCenterY)
        order,_ =  getOrder(centerX,centerY,baseAxis=True,axis=axisPrev)
        if (order[1] != 1) or (order[2] != prevOrder[2]):
            if prevOrder[0] == 2: order = swapElements(order,0,2) #if is decreasing, swap                
            sortedCenterCoord = np.array((sortedCenterCoord[order[0]], sortedCenterCoord[order[1]], sortedCenterCoord[order[2]]))
    return sortedCenterCoord, otherCamOrder


def mySVD(E):
    U,Ddiag,V = np.linalg.svd(E)
    D = np.zeros((3, 3))
    np.fill_diagonal(D, Ddiag)
    V = V.T.conj() 
    return U,D,V

def decomposeEssentialMat(E,K1,K2,pts1,pts2):
    U,D,V = mySVD(E)
    e = (D[0][0]+D[1][1])/2
    D = np.diag([e,e,0])
    E_aux = np.matmul(np.matmul(U,D),V.T)
    U,_,V = mySVD(E_aux)
    W = np.array([[0,-1,0],[1,0,0],[0,0,1]])
    Z = [[0,1,0],[-1,0,0],[0,0,0]]
    R1 = np.matmul(np.matmul(U,W),V.T)
    R2 = np.matmul(np.matmul(U,W.T),V.T)

    if np.linalg.det(R1) < 0: R1 = -R1
    if np.linalg.det(R2) < 0: R2 = -R2

    Tx = np.matmul(np.matmul(U,Z),U.T)
    t = np.array([Tx[2][1],Tx[0][2],Tx[1,0]])

    Rs = np.concatenate((R1,R1,R2,R2)).reshape(-1,3,3)
    Ts = np.concatenate((t,-t,t,-t)).reshape(-1,1,3)

    numNegatives = np.zeros((Ts.shape[0],1))
    numPoints = pts1.shape[0]
    P1 = np.hstack((K1,[[0.],[0.],[0.]]))

    for i in range(0,Ts.shape[0]):
        P2 = np.matmul(K2,np.hstack((Rs[i],Ts[i].T)))
        M1,M2 = P1[0:3, 0:3],P2[0:3, 0:3]
        c1,c2 = np.matmul(-np.linalg.inv(M1),P1[0:3,3]),np.matmul(-np.linalg.inv(M2),P2[0:3,3])
        u1,u2 = np.vstack((pts1.T,np.ones((1,numPoints)))),np.vstack((pts2.T,np.ones((1,numPoints))))
        a1,a2 = np.matmul(np.linalg.inv(M1),u1),np.matmul(np.linalg.inv(M2),u2)
        points3D,y = np.zeros((numPoints,3)),c2 - c1
        for k in range(0,numPoints):
            A = np.hstack((a1[:,k].reshape(3,1),-a2[:,k].reshape(3,1)))
            alpha = np.matmul(np.linalg.pinv(A),y)
            p = (c1 + alpha[0] * a1[:,k] + c2 + alpha[1] * a2[:,k]) / 2
            points3D[k,:] = p
        m1 = points3D
        m2 = np.add(np.matmul(m1,Rs[i].T),np.tile(Ts[i],(numPoints,1)))
        numNegatives[i] = np.sum((m1[:,2] < 0) | (m2[:,2] < 0));

    idx = numNegatives.argmin()
    R = Rs[idx]
    t = Ts[idx]
    if numNegatives.min() > 0:
        print('[ERROR] no valid rotation matrix')
        return np.NaN,np.NaN
    #t = np.matmul(-t, R)
    return R,t

def myProjectionPoints(pts):
    projPt,i = np.zeros((2, len(pts))),0
    for [x, y] in pts:
        projPt[0, i],projPt[1, i] = x,y
        i = i + 1
    return projPt

def normalisePoints(pts):
    # calculate origin centroid
    center = np.mean(pts,axis=0)
    # translate points to centroid
    traslatedPts = pts - center
    # calculate scale for the average point to be (1,1,1) >> homogeneous
    meanDist2Center = np.mean(np.linalg.norm(traslatedPts,axis=1))
    if meanDist2Center: # protect against division by zero
        scale = np.sqrt(2)/meanDist2Center
    else:
        return pts, 0, False
    
    # compute translation matrix >> (x-x_c)*scale
    T = np.diag((scale,scale,1))
    T[0:2,2] = -scale*center

    # transform in homogeneous coordinates
    homogeneousPts = np.vstack((pts.T,np.ones((1,pts.shape[0]))))
    normPoints = np.matmul(T,homogeneousPts)
    return normPoints, T, True

def erroReprojection(F,pts1,pts2):  # pts are Nx3 array of homogenous coordinates.  
    # how well F satisfies the equation pt1 * F * pt2 == 0
    vals = np.matmul(pts2.T,np.matmul(F,pts1))
    err = np.abs(vals)
    print("\t\t> avg x'Fx=0:",np.mean(err))
    print("\t\t> max x'Fx=0:",np.max(err))
    return np.mean(err)

def estimateFundMatrix_8norm(pts1,pts2,verbose=True):
    # get number of matched points
    numPoints = pts1.shape[0]
    # transform to normalized points
    normPts1,t1,valid1 = normalisePoints(pts1) 
    normPts2,t2,valid2 = normalisePoints(pts2)
    if valid1 and valid2:
        # construct A matrix for 8-norm
        # Zisserman (pag 279)
        A = np.zeros((numPoints,9))
        for i in range(numPoints):
            pt1 = normPts1[:,i]
            pt2 = normPts2[:,i]
            A[i] = [pt1[0]*pt2[0], pt1[1]*pt2[0], pt2[0],
                    pt1[0]*pt2[1], pt1[1]*pt2[1], pt2[1],
                           pt1[0],        pt1[1],      1]

        # F is the smallest singular value of A
        _,_,V = mySVD(A)
        F = V[:, -1].reshape(3,3)
        U,D,V = mySVD(F)
        D[-1,-1] = 0
        F = np.matmul(np.matmul(U,D),V.T)

        # transform F back to the original scale
        F = np.matmul(np.matmul(t2.T,F),t1)
        # normalise F
        F = F/np.linalg.norm(F)
        if F[-1,-1] < 0: F = -F
        if verbose:
            #print("Fund. Mat.\n", F.round(4))
            erroReprojection(F,np.vstack((pts1.T,np.ones((1,pts1.shape[0])))),np.vstack((pts2.T,np.ones((1,pts2.shape[0])))))
        return F,True
    else:
        return 0,False

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

def processCentroids(coord,a0,b0,cameraMatrix,distCoef):  
    undCoord = np.copy(coord)
    for i in range(0,int(coord.shape[0])):
        undCoord[i] = [undCoord[i][0]+b0-5,undCoord[i][1]+a0-5] 
    undCoord = myUndistortPointsFisheye(undCoord,cameraMatrix,distCoef)  
    return undCoord

