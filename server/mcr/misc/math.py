import numpy as np
import math
from sklearn import linear_model
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation

# VECTOR ALGEBRA

# Get the distance between points to lines
def getDistance2Line(lines,pts):
    pts,out,lines = np.copy(pts).reshape(-1,2),[],np.copy(lines).reshape(-1,3)

    for [a,b,c] in lines:
        for [x,y] in pts: out.append(abs(a*x+b*y+c)/np.sqrt(np.sqrt(pow(a,2)+pow(b,2))))

    return np.array(out)<5, np.array(out)    

# Find a plane that passes between three points
def findPlane(P1,P2,P3):
    x1,y1,z1 = P1
    x2,y2,z2 = P2
    x3,y3,z3 = P3
    a1,b1,c1 = x2-x1,y2-y1,z2-z1
    a2,b2,c2 = x3-x1,y3-y1,z3-z1
    a,b,c = b1*c2-b2*c1,a2*c1-a1*c2,a1*b2-b1*a2
    d=(-a*x1-b*y1-c*z1)
    return np.array([a,b,c,d])

# Get angle between two vectors
def getAngle(u,v):
    cosPhi = np.dot(u,v)/(np.linalg.norm(u)*np.linalg.norm(v))
    phi = np.arccos(cosPhi)
    return np.arctan2(np.sin(phi),cosPhi)

# LINEAR ALGEBRA

# FIXME: Avoid reshaping!!
def reshapeCoord(coord):
    # First reshape array of coordinates
    coordCopy = np.array(coord).reshape(6)
    
    # Get the Y coordinates of each markers' center
    coordX,coordY = [coordCopy[0], coordCopy[2], coordCopy[4]],[coordCopy[1], coordCopy[3], coordCopy[5]]

    return coordX,coordY

def normalizePoints(pts):
    # Calculate origin centroid
    center = np.mean(pts,axis=0)

    # Translate points to centroid
    traslatedPts = pts - center

    # Calculate scale for the average point to be (1,1,1) >> homogeneous
    meanDist2Center = np.mean(np.linalg.norm(traslatedPts,axis=1))
    if meanDist2Center: # Protect against division by zero
        scale = np.sqrt(2)/meanDist2Center
    else:
        return pts, 0, False
    
    # Compute translation matrix >> (x-x_c)*scale
    T = np.diag((scale,scale,1))
    T[0:2,2] = -scale*center

    # Transform in homogeneous coordinates
    homogeneousPts = np.vstack((pts.T,np.ones((1,pts.shape[0]))))
    normPoints = np.matmul(T,homogeneousPts)

    return normPoints, T, True

def singularValueDecomposition(matrix):
    leftSingVectors, singValues, rightSingVectorsTransposed = np.linalg.svd(matrix)
    singValuesMatrix = np.zeros((3, 3))
    
    np.fill_diagonal(singValuesMatrix, singValues)
    rightSingVectors = rightSingVectorsTransposed.T.conj() 
    
    return leftSingVectors, singValuesMatrix, rightSingVectors

# FITTING

def isCollinear(P1, P2, P3):
    X,y = [[P1[0]],[P2[0]],[P3[0]]],[P1[1], P2[1], P3[1]]      
    reg = linear_model.LinearRegression()       # Create LS model
    reg.fit(X, y)                               # Fit model    
    a3,b3 = reg.coef_,reg.intercept_   
    y0_LS,y1_LS,y2_LS = abs(int(a3*P1[0] + b3 - P1[1])),abs(int(a3*P2[0] + b3 - P2[1])),abs(int(a3*P3[0] + b3 - P3[1]))     
    m = (y0_LS + y1_LS + y2_LS)/3
    res = (m < 1)
    return res

# Interpolate data using cubic spline
def interpolate(coords, timestamps, steps):
    # Get data
    if not len(timestamps): 
        return [], []
    
    # Get array limits
    lowBound = math.ceil(timestamps[0] / steps)
    highBound = math.floor(timestamps[-1] / steps)
    
    # Interpolate
    newTimestamps = np.linspace(lowBound, highBound, int((highBound - lowBound)) + 1, dtype = np.uint16)
    cubicSpline = CubicSpline(timestamps, coords, axis = 0)
    
    return cubicSpline(newTimestamps * steps), newTimestamps


# UTILITY

def getSignal(N1, N2, tol=10**(-6)):
    if abs(N1-N2) <= tol: 
        return 0,False
    if (N1-N2) < 0:  
        return -1,True
    else: 
        return 1,True

def swapElements(arr, idx1, idx2):
    aux = arr[idx1]
    arr[idx1] = arr[idx2]
    arr[idx2] = aux
    return arr
