import numpy as np
from cv2.fisheye import undistortPoints

from mcr.math import normalizePoints, singularValueDecomposition

# LINEAR CAMERA MODEL

def undistortFisheye(points, intrisicsMatrix, distortionCoeffs):
    # Save variables
    pointsReshaped = points.reshape(-1, 1, 2)
    
    fx = intrisicsMatrix[0, 0]
    fy = intrisicsMatrix[1, 1]
    cx = intrisicsMatrix[0, 2]
    cy = intrisicsMatrix[1, 2]

    # Remove destortion
    undistortedPtsNorm = undistortPoints(pointsReshaped, intrisicsMatrix, distortionCoeffs)
    undistortedPtsNorm = undistortedPtsNorm.reshape(-1, 2)
    
    # Remove normalization
    undistortedPoints = np.zeros_like(undistortedPtsNorm)

    for i, (x, y) in enumerate(undistortedPtsNorm):
        px = x * fx + cx
        py = y * fy + cy

        undistortedPoints[i, 0] = px
        undistortedPoints[i, 1] = py
    
    return undistortedPoints

def reprojectionError(F,pts1,pts2):  # pts are Nx3 array of homogenous coordinates.  
    # How well F satisfies the equation pt1 * F * pt2 == 0
    vals = np.matmul(pts2.T,np.matmul(F,pts1))
    err = np.abs(vals)
    
    print("\t\t> avg x'Fx=0:",np.mean(err))
    print("\t\t> max x'Fx=0:",np.max(err))
    
    return np.mean(err)

def estimateFundMatrix_8norm(pts1,pts2,verbose=True):
    # Get number of matched points
    numPoints = pts1.shape[0]
    
    # Transform to normalized points
    normPts1,t1,valid1 = normalizePoints(pts1) 
    normPts2,t2,valid2 = normalizePoints(pts2)
    if valid1 and valid2:
        # Construct A matrix for 8-norm: Zisserman (pg. 279)
        A = np.zeros((numPoints,9))
        for i in range(numPoints):
            pt1 = normPts1[:,i]
            pt2 = normPts2[:,i]
            A[i] = [pt1[0]*pt2[0], pt1[1]*pt2[0], pt2[0],
                    pt1[0]*pt2[1], pt1[1]*pt2[1], pt2[1],
                           pt1[0],        pt1[1],      1]

        # F is the smallest singular value of A
        _,_,V = singularValueDecomposition(A)
        F = V[:, -1].reshape(3,3)
        U,D,V = singularValueDecomposition(F)
        D[-1,-1] = 0
        F = np.matmul(np.matmul(U,D),V.T)

        # Transform F back to the original scale
        
        F = np.matmul(np.matmul(t2.T,F),t1)
        
        # Normalise F
        F = F/np.linalg.norm(F)
        if F[-1,-1] < 0: 
            F = -F

        if verbose:
            reprojectionError(F,np.vstack((pts1.T,np.ones((1,pts1.shape[0])))),np.vstack((pts2.T,np.ones((1,pts2.shape[0])))))
        return F,True
    else:
        return 0,False

def decomposeEssentialMat(E,K1,K2,pts1,pts2):
    U,D,V = singularValueDecomposition(E)
    e = (D[0][0]+D[1][1])/2
    D = np.diag([e,e,0])
    E_aux = np.matmul(np.matmul(U,D),V.T)
    U,_,V = singularValueDecomposition(E_aux)
    W = np.array([[0,-1,0],[1,0,0],[0,0,1]])
    Z = [[0,1,0],[-1,0,0],[0,0,0]]
    R1 = np.matmul(np.matmul(U,W),V.T)
    R2 = np.matmul(np.matmul(U,W.T),V.T)

    if np.linalg.det(R1) < 0: 
        R1 = -R1
    if np.linalg.det(R2) < 0: 
        R2 = -R2

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
    
    return R,t

def projectionPoints(points):
    projectedPoints = np.zeros((2, len(points)))
    
    for i, (x, y) in enumerate(points):
        projectedPoints[0, i] = x
        projectedPoints[1, i] = y
        
    return projectedPoints


# Get epiline coeficients based on the fundamental matrix
def getEpilineCoef(pts,F):
    [a,b,c]=np.matmul(F,np.hstack((pts,1))) # ax + by + c = 0
    return [a,b,c]/(np.sqrt(pow(a,2)+pow(b,2)))

# Find the rotation and translation between dataframes for correction matrices
# Source: http://nghiaho.com/?page_id=671
def findRandT(ptA,ptB):
    nMarkers = min(ptA.shape[1],ptB.shape[1])
    centroidA,centroidB=np.sum(ptA,axis=1).reshape(-1,1)/nMarkers,np.sum(ptB,axis=1).reshape(-1,1)/nMarkers
    H = np.matmul(ptA-centroidA,(ptB-centroidB).T)
    [U,_,V] = singularValueDecomposition(H)
    
    R = np.matmul(V,U.T)    
    if np.linalg.det(R)<0: 
        V[:,-1] = -V[:,-1]
        R = np.matmul(V,U.T)
    
    t = centroidB-np.matmul(R,centroidA)
    
    return R,t.reshape(-1,1)

# CAMERA REFERENCE

def findOtherCamera(whichCamera,nCameras):
    if not len(whichCamera): 
        return []
    cam1,cam2 = int(whichCamera[0])-1,int(whichCamera[1])-1
    otherCam = []
    
    for i in range(nCameras):
        if i != cam1 and i != cam2: otherCam.append(i)
    
    return otherCam

# Returns if neighbor camera has captured a point at that timestamp
def getOtherValidIdx(line,nMarkers,idx,relateLast2First = 0):
    nCameras = int(line.shape[0]/2/nMarkers)
    nColumns = 2*nMarkers
    valid = [np.any(line[i*nColumns:i*nColumns+nColumns]) for i in [(idx-1)%nCameras,(idx+1)%nCameras]]
    res = np.array([(idx-1)%nCameras,(idx+1)%nCameras])[np.where(valid)[0]]
    
    if not relateLast2First and nCameras!=2: 
        if idx == 0: 
            res = np.delete(res,np.where(res == (nCameras-1))[0])
        elif idx == (nCameras-1): 
            res = np.delete(res,np.where(res == 0)[0])
    
    return res