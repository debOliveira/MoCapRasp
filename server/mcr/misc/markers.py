import numpy as np
from itertools import permutations,combinations
from scipy.spatial.distance import pdist

from mcr.misc.math import reshapeCoord, getSignal, swapElements, getDistance2Line
from mcr.misc.cameras import undistortFisheye, getEpilineCoef

def processCentroids(coord,a0,b0,cameraMatrix,distCoef):  
    undCoord = np.copy(coord)

    for i in range(0,int(coord.shape[0])):
        undCoord[i] = [undCoord[i][0]+b0-5,undCoord[i][1]+a0-5] 

    undCoord = undistortFisheye(undCoord,cameraMatrix,distCoef)  

    return undCoord

# Check if points are occluding each other 
def occlusion(pt, tol=5):
    if len(pt) > 1:
        return min(pdist(pt, metric='euclidean')) < tol
    else:
        return False
    
# Create dictionary that keeps track of ordering need with neighbor camera
def createNeedsOrder(nCameras,relateLast2First = False):
    needsOrder = {}
    for i in range(nCameras):
        # If the last camera is calibrated as a pair to the first one
        if relateLast2First:
            if not i: 
                needsOrder[str(i)]=np.array([nCameras-1,i+1])
            elif i == (nCameras-1): 
                needsOrder[str(i)]=np.array([i-1,0])
            else: 
                needsOrder[str(i)]=np.array([i-1,i+1])
        else:
            if not i: 
                needsOrder[str(i)]=np.array([i+1])
            elif i == (nCameras-1): 
                needsOrder[str(i)]=np.array([i-1])
            else: 
                needsOrder[str(i)]=np.array([i-1,i+1])

    return needsOrder

# Resets the ordering array at one camera index
def activateNeedsOrder(nCameras, idx, needsOrder, relateLast2First = False):
    if not idx:
        # Add the vector to the camera at idx
        if relateLast2First: 
            needsOrder[str(idx)]=np.array([nCameras-1,idx+1])
            needsOrder[str(nCameras-1)]=np.unique(np.hstack((needsOrder[str(nCameras-1)],[0])))
        else: 
            needsOrder[str(idx)]=np.array([idx+1])

        # Add to the neighbor camera that idx needs ordering
        needsOrder[str(idx+1)]=np.unique(np.hstack((needsOrder[str(idx+1)],[idx])))
    elif idx == (nCameras-1):
        if relateLast2First: 
            needsOrder[str(idx)]=np.array([idx-1,0])
            needsOrder[str(0)]=np.unique(np.hstack((needsOrder[0],[nCameras-1])))
        else: 
            needsOrder[str(idx)]=np.array([idx-1])

        needsOrder[str(idx-1)]=np.unique(np.hstack((needsOrder[str(idx-1)],[idx])))
    else:
        needsOrder[str(idx)]=np.array([idx-1,idx+1])
        needsOrder[str(idx+1)]=np.unique(np.hstack((needsOrder[str(idx+1)],[idx])))
        needsOrder[str(idx-1)]=np.unique(np.hstack((needsOrder[str(idx-1)],[idx])))

    return needsOrder

# Order blobs per proximity
def getTheClosest(coordNow, prev):
    # Get data into the correct shape
    centerCoord,prevCenterCoord = np.copy(coordNow).reshape(-1,2),np.copy(prev).reshape(-1,2)
    newOrder,nMarkers = np.ones(centerCoord.shape[0],dtype=np.int8)*-1,centerCoord.shape[0]

    # Order each blob
    for i in range(nMarkers):
        # Check if blob has already been ordered
        if newOrder[i] == -1:
            # Get distance from previous images blobs to actual image
            pt = prevCenterCoord[i]        
            distNow = np.linalg.norm(centerCoord-pt,axis=1)
            retNow = distNow < 5

            # If any blob is closer than 5 px, that is the one
            if np.sum(retNow) == 1: 
                newOrder[i] = np.argmin(distNow)
            else:              
                # If more than 1 one or 0 are close than 5px, get the one with the smaller distance
                allPermuationsOf4,allDists = np.array(list(permutations(list(range(0,4))))),[]

                # Get all possible distances between last and actual iamges blobs
                for idx in allPermuationsOf4:
                    newPts,dist = centerCoord[idx],0

                    for k in range(nMarkers): 
                        aux= np.linalg.norm(prevCenterCoord[k]-newPts[k])
                        dist+=aux

                    allDists.append(dist)
            
                # Get minimum distance
                minDist = np.argmin(allDists)
                choosenIdx = allPermuationsOf4[minDist]
                return choosenIdx
            
    return newOrder

# Removes indexes from neighbor camera (they have been ordered)
def popNeedsOrder(idx, otherIdx, needsOrder):
    # Delete from idx
    myVec = np.array(needsOrder[str(idx)])
    idx2Delete = np.where(myVec==otherIdx)[0]
    needsOrder[str(idx)] = np.delete(myVec,idx2Delete)

    # Delete from otherIdx
    myVec = np.array(needsOrder[str(otherIdx)])
    idx2Delete = np.where(myVec==idx)[0]
    needsOrder[str(otherIdx)] = np.delete(myVec,idx2Delete)
    return needsOrder

def getOrderPerEpiline(coord1,coord2,nMarkers,F,verbose = 0,retValue = 0):
    # Get data
    pts1,pts2,orderSecondFrame = np.copy(coord1).reshape(-1,2),np.copy(coord2).reshape(-1,2),np.ones(nMarkers,dtype=np.int8)*-1
    epilines = np.zeros((nMarkers,3))
    choosenIdx,allDists = [],[]

    for i in range(nMarkers):
        epilines[i] = getEpilineCoef(pts1[i],F)        

    allPermuationsOf4 = np.array(list(permutations(list(range(0,nMarkers)))))

    # Get all possible distances between point/line
    for idx2 in allPermuationsOf4:
        newPts2,dist = pts2[idx2],0
        for k in range(nMarkers):
            _,aux= getDistance2Line(epilines[k],newPts2[k])
            dist+=aux
        allDists.append(dist)

    # Get minimum distance
    minDist = min(allDists)

    # Get all indexes with 1 pixels distance from the mininum distance combination
    choosenIdx = allPermuationsOf4[np.where(allDists<=minDist+1)[0]]

    # Uf there are more than 1 combination possible, find ambiguous blobs
    if len(choosenIdx)>1:
        # Initiate variables
        allCombinationsOf2 = np.array(list(combinations(list(range(0,len(choosenIdx))),2)))
        mask = np.ones(nMarkers,dtype=bool)

        # Get common indexes from all ambiguous combinations
        for idx in allCombinationsOf2:
            nowMask = np.equal(choosenIdx[idx[0]],choosenIdx[idx[1]])
            mask*=nowMask

        # Invert mask to find the blobs that differs in each image
        mask = np.invert(mask)
        idxAmbiguous1,idxAmbiguous2 = np.where(mask)[0],choosenIdx[0][mask]
        collinearPts1,collinearPts2 = pts1[idxAmbiguous1],pts2[idxAmbiguous2]

        # Sort per X
        if np.all(np.diff(np.sort(collinearPts2[:,0]))>2) and np.all(np.diff(np.sort(collinearPts1[:,0]))>2):
            order1,order2 = np.argsort(collinearPts1[:,0]),np.argsort(collinearPts2[:,0])
            idx1 = np.hstack((collinearPts1,idxAmbiguous1.reshape(idxAmbiguous1.shape[0],-1)))[order1,-1].T
            idx2 = np.hstack((collinearPts2,idxAmbiguous2.reshape(idxAmbiguous2.shape[0],-1)))[order2,-1].T
        else: # Sort per Y
            order1,order2 = np.argsort(collinearPts1[:,1]),np.argsort(collinearPts2[:,1])
            idx1 = np.hstack((collinearPts1,idxAmbiguous1.reshape(idxAmbiguous1.shape[0],-1)))[order1,-1].T
            idx2 = np.hstack((collinearPts2,idxAmbiguous2.reshape(idxAmbiguous2.shape[0],-1)))[order2,-1].T

        orderSecondFrame = choosenIdx[0]
        orderSecondFrame[idx1.astype(int)] = idx2.astype(int)
    else: 
        orderSecondFrame = choosenIdx[0]
    
    if not retValue: 
        return orderSecondFrame
    if retValue: 
        return orderSecondFrame,minDist<20

def getOrder(centerX,centerY, baseAxis=False, axis = 1):
    # get wand direction
    distY,distX = np.array(centerY).max() - np.array(centerY).min(),np.array(centerX).max() - np.array(centerX).min()  
    # define the order of the markers 
    if not baseAxis:  #if there is no axis to compare, get maximum dist
        if distY > distX: 
            order,axis = np.argsort(centerY),1
        else: 
            order,axis = np.argsort(centerX),0
    else:  # if there is a previous frame, compare to its axis
        if axis: 
            order = np.argsort(centerY)
        else: 
            order = np.argsort(centerX)

    return order, axis

def findNearestC(nearestA, nearestB): # get the numer missing from the array [0,1,2]
    vec = np.array([nearestA, nearestB])
    is0, = np.where(vec == 0)
    is1, = np.where(vec == 1)
    is0,is1 = len(is0), len(is1)

    if is0:
        if is1: 
            return 2
        else: 
            return 1
    else: 
        return 0

def orderCenterCoord(centerCoord, prevCenterCoord, otherCamOrder = 0):
    centerX, centerY = reshapeCoord(centerCoord)
    # If it is the first image of the sequence

    if len(prevCenterCoord) == 0:  
        order,_ =  getOrder(centerX,centerY)  
        # If it is the second camera
        
        if otherCamOrder != 0:  
            # If the markers are wrong, swap the extremities
            signal, valid = getSignal(centerX[order[0]], centerX[order[2]],5)
            if signal != otherCamOrder and valid: order = swapElements(order, 0, 2)    
        else:        
            # Get base for comparision (first camera only)        
            otherCamOrder,_ = getSignal(centerX[order[0]], centerX[order[2]])
        
        # Sort centers        
        if np.linalg.norm(centerX[order[0]]-centerX[order[1]])>np.linalg.norm(centerX[order[2]]-centerX[order[1]]):
            sortedCenterCoord = np.array((centerCoord[order[0]], centerCoord[order[1]], centerCoord[order[2]]))
        else: 
            sortedCenterCoord = np.array((centerCoord[order[2]], centerCoord[order[1]], centerCoord[order[0]]))
    else:
        # First reshape array of coordinates
        prevCenterX,prevCenterY = reshapeCoord(prevCenterCoord)
        
        # Distance from marker A/B of previous img to center coordiantes of actual img
        distA = np.sqrt(np.power(np.subtract(prevCenterX[0], centerX), 2) + np.power(np.subtract(prevCenterY[0], centerY), 2))
        distB = np.sqrt(np.power(np.subtract(prevCenterX[1], centerX), 2) + np.power(np.subtract(prevCenterY[1], centerY), 2))
        
        # Nearest marker from A is selected and removed as marker B candidate
        nearestA = np.argmin(distA)
        distBCopy = np.delete(distB, nearestA)
        
        # Narest marker from B is selected and removed as marker C candidate
        nearestBCopy = np.argmin(distBCopy)
        nearestB, = np.where(distB == distBCopy[nearestBCopy])
        distBCopy = np.delete(distBCopy, nearestBCopy)
        
        # Get the missing marker position in array
        nearestC = findNearestC(nearestA, nearestB[0])
        
        # Sort centers        
        sortedCenterCoord = [centerCoord[nearestA], centerCoord[nearestB[0]], centerCoord[nearestC]]
        
        # Check if the ordering is ok
        centerX, centerY = reshapeCoord(sortedCenterCoord)
        prevOrder,axisPrev = getOrder(prevCenterX,prevCenterY)
        order,_ =  getOrder(centerX,centerY,baseAxis=True,axis=axisPrev)
        
        if (order[1] != 1) or (order[2] != prevOrder[2]):
            if prevOrder[0] == 2: order = swapElements(order,0,2) #if is decreasing, swap                
            sortedCenterCoord = np.array((sortedCenterCoord[order[0]], sortedCenterCoord[order[1]], sortedCenterCoord[order[2]]))
    
    return sortedCenterCoord, otherCamOrder

# TODO: Where is this applied ???
def getPreviousCentroid(noPrevious, lastCentroid):
    if not noPrevious: 
        return []
    else: 
        return lastCentroid






