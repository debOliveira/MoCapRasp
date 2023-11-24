# IMPORTS >>> DO NOT CHANGE <<<
import warnings
warnings.filterwarnings('ignore')
import os, math
from datetime import datetime
import numpy as np
from scipy.interpolate import CubicSpline
from cv2 import destroyAllWindows, triangulatePoints

from mcr.capture.CaptureProcess import CaptureProcess

from mcr.misc.math import isCollinear
from mcr.misc.cameras import estimateFundMatrix_8norm, decomposeEssentialMat, projectionPoints
from mcr.misc.markers import orderCenterCoord, occlusion, processCentroids
from mcr.misc.plot import plotArena

class CEC(CaptureProcess):
    # Collect points from clients, order and trigger interpolation for Camera Extrinsics Calibration
    def collect(self):
        print('[INFO] waiting capture')
       
        # Internal variables
        capture = np.ones(self.cameras,dtype=np.bool)
        counter,lastTime = np.zeros(self.cameras,dtype=np.uint16),np.zeros(self.cameras,dtype=np.uint32)
        missed,invalid = np.zeros(self.cameras,dtype=np.uint32),np.zeros(self.cameras,dtype=np.uint32)
        swap,certainty = np.zeros(self.cameras,dtype=np.uint16),np.zeros(self.cameras,dtype=np.bool8)
        lastImgNumber,tol = np.zeros(self.cameras,dtype=np.int32),0.25
        intervals,timeIntervals,dfSave,dfOrig = [],[],[],[]
        rotation,translation,scale,FMatrix,points3D_perPair, = [np.identity(3)],[[[0., 0., 0.]]],[[1]],[],[]

        for i in range(self.cameras): # For each camera
            intervals.append([]) # List for each camera
            timeIntervals.append([]) # List for each camera
            dfOrig.append([]) # List for each camera

        # Capture loop
        try:
            while np.any(capture):
                # Receive message
                bytesPair = self.server_socket.recvfrom(self.bufferSize)
                message = np.frombuffer(bytesPair[0],dtype=np.float64)
                address,sizeMsg = bytesPair[1],len(message)
                idx = self.ipList.index(address[0])
                if not (sizeMsg-1): capture[idx] = 0

                # If valid message
                if capture[idx]: # Check if message is valid
                    if sizeMsg < 13: # If less than 3 blobs, discard
                        if self.verbose: print('[ERROR] '+str(int((sizeMsg-4)/3))+' markers were found')
                        missed[idx]+=1
                    else: 
                        msg = message[0:sizeMsg-4].reshape(-1,3)
                        coord,size = msg[:,0:2],msg[:,2].reshape(-1)

                        # If more than 3 blobs are found, get the four biggest
                        if sizeMsg > 13: 
                            orderAscDiameters = np.argsort(size)
                            coord = np.array([coord[orderAscDiameters[-1]],coord[orderAscDiameters[-2]],coord[orderAscDiameters[-3]]]).reshape(-1,2)

                        # Store message parameters
                        a,b,time,imgNumber = message[-4],message[-3],message[-2],int(message[-1]) 

                        # Undistort points
                        undCoord = processCentroids(coord,a,b,self.cameraMat[idx],self.distCoef[idx])
                        
                        if self.save: dfSave.append(np.concatenate((undCoord.reshape(6),[time,imgNumber,idx])))

                        # If ts if not read corectly, discard
                        if counter[idx]:
                            if abs(time-lastTime[idx])>1e9: 
                                if self.verbose: print('[WARNING-CAM+'+str(idx)+'] time missmatch')
                                missed[idx]+=1
                                invalid[idx]+=1
                                continue

                        # Check if sequence is valid
                        if imgNumber > lastImgNumber[idx]+1: invalid[idx] = imgNumber-lastImgNumber[idx]

                        # Order markers per proximity and check collinearity
                        if isCollinear(*undCoord) and not occlusion(undCoord,5) and not np.any(undCoord<0):     
                            if invalid[idx] >= 10 or not counter[idx]: 
                                if certainty[idx]:
                                    beg,end = intervals[idx][-1],counter[idx]-1
                                    timeIntervals[idx].append([dfOrig[idx][beg,6],dfOrig[idx][end,6]])
                                    if self.verbose: print('[WARNING-CAM+'+str(idx)+'] valid from '+str(round(dfOrig[idx][beg,6]/1e6,2))+'s to '+str(round(dfOrig[idx][end][6]/1e6,2))+'s')
                                prev,certainty[idx] = [],False
                                intervals[idx].append(counter[idx])
                            else: 
                                if not (counter[idx]-1): prev = np.array(dfOrig[idx][0:6]).reshape(-1,2)
                                else: prev = np.array(dfOrig[idx][-1,0:6]).reshape(-1,2)
                            undCoord, _ = orderCenterCoord(undCoord,prev)
                            undCoord = np.array(undCoord)
                        else: 
                            if self.verbose: print('[WARNING-CAM+'+str(idx)+'] not collinear or equal centroids')
                            missed[idx]+=1
                            invalid[idx]+=1
                            continue

                        # Update loop variables
                        lastTime[idx],lastImgNumber[idx],invalid[idx] = time,imgNumber,0
                        if not counter[idx]: dfOrig[idx] = np.hstack((undCoord.reshape(6),time))
                        else: dfOrig[idx] = np.vstack((dfOrig[idx],np.hstack((undCoord.reshape(6),time))))
                        counter[idx]+=1

                        # Check if ABC is in order smaller to largest
                        if not certainty[idx]:
                            for [A,B,C] in undCoord.reshape([-1, 3, 2]):
                                if np.linalg.norm(A-B)/np.linalg.norm(C-B)>(2-tol) and np.linalg.norm(A-B)>20:
                                    swap[idx] += 1
                                    if swap[idx]>2:    
                                        swap[idx],certainty[idx] = 0,True
                                        dfOrig[idx][intervals[idx][-1]:counter[idx],0:2],dfOrig[idx][intervals[idx][-1]:counter[idx],4:6] = np.copy(dfOrig[idx][intervals[idx][-1]:counter[idx],4:6]),np.copy(dfOrig[idx][intervals[idx][-1]:counter[idx],0:2])
                                if np.linalg.norm(C-B)/np.linalg.norm(A-B)>(2-tol) and np.linalg.norm(C-B)>20:  certainty[idx] = True
                                            
        finally:        
            # Close everything
            self.server_socket.close()
            destroyAllWindows()

            # Save Camera Extrinsics Calibration (CEC) Data
            if self.save: 
                now = datetime.now()
                ymd, HMS = now.strftime('%y-%m-%d'), now.strftime('%H-%M-%S')
                path = 'debug/dataSaves/' + ymd + '/'

                # Check whether directory already exists
                if not os.path.exists(path):
                    os.mkdir(path)
                    print(f'Folder {path} created!')
        
                np.savetxt(path + 'CEC-' + HMS + '.csv', np.array(dfSave), delimiter=',')
            
            # Get last interval
            for idx in range(self.cameras):
                if not len(dfOrig[idx]): continue
                if certainty[idx]:
                    beg,end = intervals[idx][-1],counter[idx]-1
                    timeIntervals[idx].append([dfOrig[idx][beg,6],dfOrig[idx][end,6]])    
                    if self.verbose: print('[WARNING-CAM+'+str(idx)+'] valid from '+str(round(dfOrig[idx][beg,6]/1e6,2))+'s to '+str(round(dfOrig[idx][end][6]/1e6,2))+'s')
            
            # Print results
            print('[RESULTS] server results are')
            for i in range(self.cameras): print('  >> camera '+str(i)+': '+str(len(dfOrig[i]))+' valid images, address '+str(self.ipList[i])+', missed '+str(int(missed[i]))+' images')
            
            # Get pose between each pair
            for cam in range(self.cameras-1):
                # Compute valid time intersection for interpolation
                intersections = [[max(first[0], second[0]), min(first[1], second[1])]  
                                    for first in timeIntervals[cam] for second in timeIntervals[cam+1]  
                                    if max(first[0], second[0]) <= min(first[1], second[1])]
                
                # Create and fill inteprolation dataset based on the intersection of valid time intervals 
                dfInterp = np.zeros((self.nImages,2*6+1))
                dfInterp[:,-1] = np.arange(0,self.record,self.step)
                for [beg,end] in intersections:
                    for idx in range(cam,cam+2):
                        validIdx = [i for i in range(0,len(dfOrig[idx])) if beg<=dfOrig[idx][i,-1]<=end]
                        coord,time = dfOrig[idx][validIdx,0:6],dfOrig[idx][validIdx,6]/1e6
                        if time.shape[0]<=2: continue
                        lowBound,highBound = math.ceil(time[0]/self.step),math.floor(time[-1]/self.step)
                        if self.verbose: print('[INFO] interpolated #'+str(idx)+' from '+str(round(lowBound*self.step,2))+'s to '+str(round(highBound*self.step,2))+'s')
                        tNew = np.linspace(lowBound,highBound,int((highBound-lowBound))+1,dtype=np.uint16)
                        ff = CubicSpline(time,coord,axis=0)
                        dfInterp[tNew,int((idx-cam)*6):int((idx-cam)*6+6)] = ff(tNew*self.step)

                # Get data from the interpolation dataset
                dfInterp = np.delete(dfInterp,np.unique([i for i in range(0,dfInterp.shape[0]) for idx in range(2) if not np.any(dfInterp[i][idx*6:idx*6+6])]),axis=0)
                if dfInterp.shape[0] < 10: 
                    print('[ERROR] no valid image intersection for cameras '+str(cam)+' and '+str(cam+1))
                    return
                centroids1,centroids2 = dfInterp[:,0:6].reshape(-1,2),dfInterp[:,6:12].reshape(-1,2)
                print('[WARNING] interpolated '+str(dfInterp.shape[0])+' images between cameras '+str(cam)+' and '+str(cam+1))

                # Get fundamental and essential matrices 
                print('[INFO] Computing fundamental and essential matrix between cameras '+str(cam)+'-'+str(cam+1))
                try: 
                    F,_ = estimateFundMatrix_8norm(np.array(centroids1),np.array(centroids2),verbose=False)
                    E = np.matmul(self.cameraMat[cam+1].T, np.matmul(F, self.cameraMat[cam]))

                    # Decompose to rotation and translation between cameras
                    R, t = decomposeEssentialMat(E, self.cameraMat[cam], self.cameraMat[cam+1], np.array(centroids1), np.array(centroids2))
                    if np.any(np.isnan(R)): 
                        print('[ERROR] no valid rotation matrix')
                        return
                    else:
                        if self.verbose:
                            print('\nRot. Mat.\n', R.round(4))
                            print('\nTrans. Mat.\n', t.round(4))
                except: 
                    print('[ERROR] no valid rotation matrix')
                    return
                
                # Create projection matrices and triangulate to compute scale
                P1,P2 = np.hstack((self.cameraMat[cam], [[0.], [0.], [0.]])),np.matmul(self.cameraMat[cam+1], np.hstack((R, t.T)))
                projPt1,projPt2 = projectionPoints(np.array(centroids1)),projectionPoints(np.array(centroids2))
                points4d = triangulatePoints(P1.astype(float),P2.astype(float),projPt1.astype(float),projPt2.astype(float))
                points3d = (points4d[:3, :]/points4d[3, :]).T
                if points3d[0, 2] < 0: points3d = -points3d
                tot,L_real_AC,L_real_AB,L_real_BC,k,false_idx = 0,15.7,5.5,10.2,0,[]
                
                # Compute std deviation and plot beautiful stuff
                for [A, B, C] in points3d.reshape([-1, 3, 3]):
                    L_rec_AC,L_rec_BC,L_rec_AB = np.linalg.norm(A-C),np.linalg.norm(B-C),np.linalg.norm(A-B)
                    tot = tot + L_real_AC/L_rec_AC + L_real_BC/L_rec_BC + L_real_AB/L_rec_AB
                    k+=3
                lamb = tot/k
                points3d_new,i,k= points3d*lamb,0,0
                for [A, B, C] in points3d_new.reshape([-1, 3, 3]):
                    L_reconst = np.sqrt(np.sum((A-C)**2, axis=0))
                    valid = abs(L_real_AC-L_reconst)/L_real_AC < 0.01
                    if not valid: 
                        i+=1
                        false_idx.extend((k,k+1,k+2))
                    k+=3

                print('\tImages distant more than 1% from the real value = ' + str(i)+'/'+str(int(points3d.shape[0]/3)))
                
                # Deleting points and refining estimation of the fundamental and essential matrices
                print('[INFO] Refining fundamental matrix estimation')
                centroids1,centroids2=np.delete(np.copy(centroids1),false_idx,axis=0),np.delete(np.copy(centroids2),false_idx,axis=0)
                
                # Get fundamental and essential matrices
                F,_ = estimateFundMatrix_8norm(np.copy(centroids1),np.copy(centroids2))
                E = np.matmul(self.cameraMat[cam+1].T, np.matmul(F, self.cameraMat[cam]))
                
                # Decompose to rotation and translation between cameras
                R, t = decomposeEssentialMat(E, self.cameraMat[cam], self.cameraMat[cam+1], np.copy(centroids1), np.copy(centroids2))
                if np.any(np.isnan(R)): 
                    print('[ERROR] no valid rotation matrix')
                    return
                else:
                    if self.verbose:
                        print('\nRot. Mat.\n', R.round(4))
                        print('\nTrans. Mat.\n', t.round(4))

                P1,P2 = np.hstack((self.cameraMat[cam], [[0.], [0.], [0.]])),np.matmul(self.cameraMat[cam+1], np.hstack((R, t.T)))
                projPt1,projPt2 = projectionPoints(np.copy(centroids1)),projectionPoints(np.copy(centroids2))
                points4d = triangulatePoints(P1.astype(float),P2.astype(float),projPt1.astype(float),projPt2.astype(float))
                points3d = (points4d[:3, :]/points4d[3, :]).T

                if points3d[0, 2] < 0: 
                    points3d = -points3d
                tot,L_AC_vec,L_BC_vec,L_AB_vec,k = 0,[],[],[],0
                
                # Compute sdt deviation and plot beautiful stuff
                for [A, B, C] in points3d.reshape([-1, 3, 3]):
                    L_rec_AC,L_rec_BC,L_rec_AB = np.linalg.norm(A-C),np.linalg.norm(B-C),np.linalg.norm(A-B)
                    tot = tot + L_real_AC/L_rec_AC + L_real_BC/L_rec_BC + L_real_AB/L_rec_AB
                    k+=3
                    L_AC_vec.append(L_rec_AC)
                    L_BC_vec.append(L_rec_BC)
                    L_AB_vec.append(L_rec_AB)
                lamb = tot/k

                print('\tScale between real world and triang. point cloud is: ', lamb.round(2))
                print('\tL_AC >> mean = ' + str((np.mean(L_AC_vec)*lamb).round(4)) +
                    'cm, std. dev = ' + str((np.std(L_AC_vec)*lamb).round(4)) +
                    'cm, rms = ' + str((np.sqrt(np.mean(np.square(np.array(L_AC_vec)*lamb-L_real_AC)))).round(4)) + 'cm')
                print('\tL_AB >> mean = ' + str((np.mean(L_AB_vec)*lamb).round(4)) +
                    'cm, std. dev = ' + str((np.std(L_AB_vec)*lamb).round(4)) +
                    'cm, rms = ' + str((np.sqrt(np.mean(np.square(np.array(L_AB_vec)*lamb-L_real_AB)))).round(4)) + 'cm')
                print('\tL_BC >> mean = ' + str((np.mean(L_BC_vec)*lamb).round(4)) +
                    'cm, std. dev = ' + str((np.std(L_BC_vec)*lamb).round(4)) +
                    'cm, rms = ' + str((np.sqrt(np.mean(np.square(np.array(L_BC_vec)*lamb-L_real_BC)))).round(4)) + 'cm')
                
                translation.append(t)
                rotation.append(R)
                scale.append([lamb])
                FMatrix.append(F)
                points3D_perPair.append(points3d)
            
            # Saving csv
            np.savetxt('mcr/capture/data/R.csv', np.ravel(rotation), delimiter=',')
            np.savetxt('mcr/capture/data/t.csv', np.ravel(translation), delimiter=',')
            np.savetxt('mcr/capture/data/lamb.csv', np.ravel(scale), delimiter=',')
            np.savetxt('mcr/capture/data/F.csv', np.ravel(FMatrix), delimiter=',')
            
            # Initializing arrays
            allPoints3d,projMat = [],[]           

            # Getting coordinates from each camera and 3D points in relation to the 0th camera
            for cam in range(self.cameras):
                # Initialize initial values of the projection matrices
                P_new = np.vstack((np.hstack((np.identity(3),np.zeros((3,1)))),np.hstack((np.zeros((3)),1))))

                # Iterate over the previous cameras to find the relation to the 0th camera
                # E.g. 3 -> 2 -> 1 -> 0
                for i in np.flip(range(cam+1)):
                    t,R,lamb = np.array(translation[i][0]).reshape(-1,3),np.array(rotation[i]),scale[i]
                    t_new = np.matmul(-t, R).reshape(-1,3)*lamb/100
                    P = np.vstack((np.hstack((R.T,t_new.T)),np.hstack((np.zeros((3)),1))))
                    P_new = np.matmul(P,P_new)

                # Save new projection matrix
                projMat.append(P_new)

                # Save triangulated 3D points in relation to the 0th camera
                if cam < self.cameras-1:
                    points3d = np.hstack((points3D_perPair[cam]*scale[cam+1][0]/100,np.ones((points3D_perPair[cam].shape[0],1)))).T
                    points3d = np.matmul(P_new,points3d)
                    if not len(allPoints3d): 
                        allPoints3d = points3d.copy()
                    else: 
                        allPoints3d = np.hstack((allPoints3d,points3d.copy()))

            # Saving projection matrices
            np.savetxt('mcr/capture/data/projMat.csv', np.array(projMat).ravel(), delimiter=',')

            # Prepare camera data
            cameraData = {'projectionMatrices': projMat}

            plotArena(title='3D Map of the Calibration Process', 
                      allPoints3d=allPoints3d, 
                      cameraData=cameraData)