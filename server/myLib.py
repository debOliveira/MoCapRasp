import numpy as np
from sklearn import linear_model
from cv2.fisheye import undistortPoints
from itertools import permutations,combinations
from cv2 import COLOR_GRAY2RGB,cvtColor,line,circle,FONT_HERSHEY_SIMPLEX,putText
import math
from scipy.interpolate import CubicSpline

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

# check if there is occlusion
def isEqual4(pt,tol=5):
    A,B,C,D = pt[0],pt[1],pt[2],pt[3]
    AB,AC,BC = np.linalg.norm(A-B),np.linalg.norm(A-C),np.linalg.norm(C-B)
    AD,BD,CD = np.linalg.norm(A-D),np.linalg.norm(B-D),np.linalg.norm(C-D)
    return min(AB,AC,BC,AD,BD,CD)<tol

# get epiline coeficients based on the fundamental matrix
def getEpilineCoef(pts,F):
    [a,b,c]=np.matmul(F,np.hstack((pts,1))) #ax+by+c=0
    return [a,b,c]/(np.sqrt(pow(a,2)+pow(b,2)))

# get the distance between a point and a line
def getDistance2Line(lines,pts):
    pts,out,lines = np.copy(pts).reshape(-1,2),[],np.copy(lines).reshape(-1,3)
    for [a,b,c] in lines:
        for [x,y] in pts: out.append(abs(a*x+b*y+c)/np.sqrt(np.sqrt(pow(a,2)+pow(b,2))))
    return np.array(out)<5,np.array(out)    

# find a plane that passes between three points
def findPlane(A,C,D):
    x1,y1,z1 = A
    x2,y2,z2 = C
    x3,y3,z3 = D
    a1,b1,c1 = x2-x1,y2-y1,z2-z1
    a2,b2,c2 = x3-x1,y3-y1,z3-z1
    a,b,c = b1*c2-b2*c1,a2*c1-a1*c2,a1*b2-b1*a2
    d=(-a*x1-b*y1-c*z1)
    return np.array([a,b,c,d])

# draw epipolar lines with numbers
def drawlines(img1,img2,lines,pts1,pts2):    
    r,c = img1.shape
    img1 = cvtColor(img1.astype('float32'),COLOR_GRAY2RGB)
    img2 = cvtColor(img2.astype('float32'),COLOR_GRAY2RGB)
    listColors = [(0,0,255),(0,255,0),(255,0,0),(255,0,255)]
    i = 0
    for r,pt1,pt2 in zip(lines,pts1,pts2):
        color = listColors[i]
        x0,y0 = map(int, [0, -r[2]/r[1] ])
        x1,y1 = map(int, [c, -(r[2]+r[0]*c)/r[1] ])
        img1 = line(img1, (x0,y0), (x1,y1), color,1)
        img1 = circle(img1,tuple(pt1),5,color,-1)
        putText(img1,str(i),tuple(pt1-20),FONT_HERSHEY_SIMPLEX,0.5,color,2) 
        img2 = circle(img2,tuple(pt2),5,color,-1)
        putText(img2,str(i),tuple(pt2-20),FONT_HERSHEY_SIMPLEX,0.5,color,2) 
        i+=1
    return img1,img2

# ordering approach using epiline proximity
def getOrderPerEpiline(coord1,coord2,nMarkers,F,verbose = 0,retValue = 0):
    # get data
    pts1,pts2,orderSecondFrame = np.copy(coord1).reshape(-1,2),np.copy(coord2).reshape(-1,2),np.ones(nMarkers,dtype=np.int8)*-1
    epilines = np.zeros((nMarkers,3))
    choosenIdx,allDists = [],[]
    for i in range(nMarkers):
        epilines[i] = getEpilineCoef(pts1[i],F)        
    allPermuationsOf4 = np.array(list(permutations(list(range(0,nMarkers)))))
    # get all possible distances between point/line
    for idx2 in allPermuationsOf4:
        newPts2,dist = pts2[idx2],0
        for k in range(nMarkers):
            _,aux= getDistance2Line(epilines[k],newPts2[k])
            dist+=aux
        allDists.append(dist)
    # get minimum distance
    minDist = min(allDists)
    # get all indexes with 1 pixels distance from the mininum distance combination
    choosenIdx = allPermuationsOf4[np.where(allDists<=minDist+1)[0]]
    # if there are more than 1 combination possible, find ambiguous blobs
    if len(choosenIdx)>1:
        # initiate variables
        allCombinationsOf2 = np.array(list(combinations(list(range(0,len(choosenIdx))),2)))
        mask = np.ones(nMarkers,dtype=np.bool)
        # get common indexes from all ambiguous combinations
        for idx in allCombinationsOf2:
            nowMask = np.equal(choosenIdx[idx[0]],choosenIdx[idx[1]])
            mask*=nowMask
        # invert mask to find the blobs that differs in each image
        mask = np.invert(mask)
        idxAmbiguous1,idxAmbiguous2 = np.where(mask)[0],choosenIdx[0][mask]
        collinearPts1,collinearPts2 = pts1[idxAmbiguous1],pts2[idxAmbiguous2]
        # sort per X
        if np.all(np.diff(np.sort(collinearPts2[:,0]))>2) and np.all(np.diff(np.sort(collinearPts1[:,0]))>2):
            order1,order2 = np.argsort(collinearPts1[:,0]),np.argsort(collinearPts2[:,0])
            idx1 = np.hstack((collinearPts1,idxAmbiguous1.reshape(idxAmbiguous1.shape[0],-1)))[order1,-1].T
            idx2 = np.hstack((collinearPts2,idxAmbiguous2.reshape(idxAmbiguous2.shape[0],-1)))[order2,-1].T
        else: # sort per Y
            order1,order2 = np.argsort(collinearPts1[:,1]),np.argsort(collinearPts2[:,1])
            idx1 = np.hstack((collinearPts1,idxAmbiguous1.reshape(idxAmbiguous1.shape[0],-1)))[order1,-1].T
            idx2 = np.hstack((collinearPts2,idxAmbiguous2.reshape(idxAmbiguous2.shape[0],-1)))[order2,-1].T
        orderSecondFrame = choosenIdx[0]
        orderSecondFrame[idx1.astype(int)] = idx2.astype(int)
    else: orderSecondFrame = choosenIdx[0]
    
    if not retValue: return orderSecondFrame
    if retValue: return orderSecondFrame,minDist<20

# find the rotation and translation between dataframes
# source: http://nghiaho.com/?page_id=671
def findRandT(ptA,ptB):
    nMarkers = min(ptA.shape[1],ptB.shape[1])
    centroidA,centroidB=np.sum(ptA,axis=1).reshape(-1,1)/nMarkers,np.sum(ptB,axis=1).reshape(-1,1)/nMarkers
    H = np.matmul(ptA-centroidA,(ptB-centroidB).T)
    [U,_,V] = mySVD(H)
    R = np.matmul(V,U.T)    
    if np.linalg.det(R)<0: 
        V[:,-1] = -V[:,-1]
        R = np.matmul(V,U.T)
    t = centroidB-np.matmul(R,centroidA)
    return R,t.reshape(-1,1)

def findOtherCamera(whichCamera,nCameras):
    if not len(whichCamera): return []
    cam1,cam2 = int(whichCamera[0])-1,int(whichCamera[1])-1
    otherCam = []
    for i in range(nCameras):
        if i != cam1 and i != cam2: otherCam.append(i)
    return otherCam


# order blobs per proximity
def getTheClosest(coordNow, prev):
    # get data into the correct shape
    centerCoord,prevCenterCoord = np.copy(coordNow).reshape(-1,2),np.copy(prev).reshape(-1,2)
    newOrder,nMarkers = np.ones(centerCoord.shape[0],dtype=np.int8)*-1,centerCoord.shape[0]
    # order each blob
    for i in range(nMarkers):
        # check if blob has already been ordered
        if newOrder[i] == -1:
            # get distance from previous images blobs to actual image
            pt = prevCenterCoord[i]        
            distNow = np.linalg.norm(centerCoord-pt,axis=1)
            retNow = distNow < 5
            # if any blob is closer than 5 px, that is the one
            if np.sum(retNow) == 1: newOrder[i] = np.argmin(distNow)
            else:              
                # if more than 1 one or 0 are close than 5px, get the one with the smaller distance
                allPermuationsOf4,allDists = np.array(list(permutations(list(range(0,4))))),[]
                # get all possible distances between last and actual iamges blobs
                for idx in allPermuationsOf4:
                    newPts,dist = centerCoord[idx],0
                    for k in range(nMarkers): 
                        aux= np.linalg.norm(prevCenterCoord[k]-newPts[k])
                        dist+=aux
                    allDists.append(dist)
                # get minimum distance
                minDist = np.argmin(allDists)
                choosenIdx = allPermuationsOf4[minDist]
                return choosenIdx
    return newOrder

# interpolate data using cubic spline
def myInterpolate(coord,ts,step):
    # get data
    if not len(ts): return [],[]
    # get array limits
    lowBound,highBound = math.ceil(ts[0]/step),math.floor(ts[-1]/step)
    # interpolate
    tNew = np.linspace(lowBound,highBound,int((highBound-lowBound))+1,dtype=np.uint16)
    ff = CubicSpline(ts,coord,axis=0)
    return ff(tNew*step),tNew

# returns if neighbor camera has captured a point at that timestamp
def getOtherValidIdx(line,nMarkers,idx,relateLast2First = 0):
    nCameras = int(line.shape[0]/2/nMarkers)
    nColumns = 2*nMarkers
    valid = [np.any(line[i*nColumns:i*nColumns+nColumns]) for i in [(idx-1)%nCameras,(idx+1)%nCameras]]
    res = np.array([(idx-1)%nCameras,(idx+1)%nCameras])[np.where(valid)[0]]
    if not relateLast2First and nCameras!=2: 
        if idx == 0: res = np.delete(res,np.where(res == (nCameras-1))[0])
        elif idx == (nCameras-1): res = np.delete(res,np.where(res == 0)[0])
    return res

# create dictionary that keeps track of ordering need with neighbor camera
def createNeedsOrder(nCameras,relateLast2First = 0):
    needsOrder = {}
    for i in range(nCameras):
        # if the last camera is calibrated as a pair to the first one
        if relateLast2First:
            if not i: needsOrder[str(i)]=np.array([nCameras-1,i+1])
            elif i == (nCameras-1): needsOrder[str(i)]=np.array([i-1,0])
            else: needsOrder[str(i)]=np.array([i-1,i+1])
        else:
            if not i: needsOrder[str(i)]=np.array([i+1])
            elif i == (nCameras-1): needsOrder[str(i)]=np.array([i-1])
            else: needsOrder[str(i)]=np.array([i-1,i+1])
    return needsOrder

# resets the ordering array at one camera index
def activateNeedsOrder(nCameras, idx, needsOrder, relateLast2First = 0):
    if not idx:
        # add the vector to the camera at idx
        if relateLast2First: 
            needsOrder[str(idx)]=np.array([nCameras-1,idx+1])
            needsOrder[str(nCameras-1)]=np.unique(np.hstack((needsOrder[str(nCameras-1)],[0])))
        else: needsOrder[str(idx)]=np.array([idx+1])
        # add to the neighbor camera that idx needs ordering
        needsOrder[str(idx+1)]=np.unique(np.hstack((needsOrder[str(idx+1)],[idx])))
    elif idx == (nCameras-1):
        if relateLast2First: 
            needsOrder[str(idx)]=np.array([idx-1,0])
            needsOrder[str(0)]=np.unique(np.hstack((needsOrder[0],[nCameras-1])))
        else: needsOrder[str(idx)]=np.array([idx-1])
        needsOrder[str(idx-1)]=np.unique(np.hstack((needsOrder[str(idx-1)],[idx])))
    else:
        needsOrder[str(idx)]=np.array([idx-1,idx+1])
        needsOrder[str(idx+1)]=np.unique(np.hstack((needsOrder[str(idx+1)],[idx])))
        needsOrder[str(idx-1)]=np.unique(np.hstack((needsOrder[str(idx-1)],[idx])))
    return needsOrder

# removes indexes from neighbor camera (they have been ordered)
def popNeedsOrder(idx, otherIdx, needsOrder):
    # delete from idx
    myVec = np.array(needsOrder[str(idx)])
    idx2Delete = np.where(myVec==otherIdx)[0]
    needsOrder[str(idx)] = np.delete(myVec,idx2Delete)
    # delete from otherIdx
    myVec = np.array(needsOrder[str(otherIdx)])
    idx2Delete = np.where(myVec==idx)[0]
    needsOrder[str(otherIdx)] = np.delete(myVec,idx2Delete)
    return needsOrder

# get angle between two vectors
def getAngle(a,b):
    cosPhi = np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b))
    phi = np.arccos(cosPhi)
    return np.arctan2(np.sin(phi),cosPhi)

