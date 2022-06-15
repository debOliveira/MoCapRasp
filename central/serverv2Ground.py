# IMPORTS >>> DO NOT CHANGE <<<
import warnings
warnings.filterwarnings("ignore")
import socket,time,argparse
import numpy as np
from cv2 import circle,COLOR_GRAY2RGB,destroyAllWindows,triangulatePoints,cvtColor,line,computeCorrespondEpilines
from myLib import myProjectionPoints,processCentroids
from constants import cameraMat,distCoef
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from itertools import permutations,combinations

def isEqual4(pt,tol=5):
    A,B,C,D = pt[0],pt[1],pt[2],pt[3]
    AB,AC,BC = np.linalg.norm(A-B),np.linalg.norm(A-C),np.linalg.norm(C-B)
    AD,BD,CD = np.linalg.norm(A-D),np.linalg.norm(B-D),np.linalg.norm(C-D)
    return min(AB,AC,BC,AD,BD,CD)<tol

def getEpilineCoef(pts,F):
    [a,b,c]=np.matmul(F,np.hstack((pts,1))) #ax+by+c=0
    return [a,b,c]/(np.sqrt(pow(a,2)+pow(b,2)))

def getDistance2Line(lines,pts):
    pts,out,lines = np.copy(pts).reshape(-1,2),[],np.copy(lines).reshape(-1,3)
    for [a,b,c] in lines:
        for [x,y] in pts: out.append(abs(a*x+b*y+c)/np.sqrt(np.sqrt(pow(a,2)+pow(b,2))))
    return np.array(out)<5,np.array(out)   

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
        img2 = circle(img2,tuple(pt2),5,color,-1)
        i+=1
    return img1,img2

def findPlane(A,C,D):
    x1,y1,z1 = A
    x2,y2,z2 = C
    x3,y3,z3 = D
    a1,b1,c1 = x2-x1,y2-y1,z2-z1
    a2,b2,c2 = x3-x1,y3-y1,z3-z1
    a,b,c = b1*c2-b2*c1,a2*c1-a1*c2,a1*b2-b1*a2
    d=(-a*x1-b*y1-c*z1)
    return np.array([a,b,c,d])

def getOrderPerEpiline(coord1,coord2,nMarkers,F,verbose = 0):
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
    # get all idx with 1 pixels distance from the mininum distance combination
    choosenIdx = allPermuationsOf4[np.where(allDists<=minDist+1)[0]]
    # id there are more than 1 combination possible, find ambiguous blobs
    if len(choosenIdx)>1:
        # initiate variables
        allCombinationsOf2 = np.array(list(combinations(list(range(0,len(choosenIdx))),2)))
        mask = np.ones(nMarkers,dtype=np.bool)
        # get common idx from all ambiguous combinations
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

    ## verbose
    if verbose:
        pts2 = np.copy(pts2[orderSecondFrame])
        img1,img2 = np.ones((720,960))*255,np.ones((720,960))*255
        lines1 = computeCorrespondEpilines(np.int32(pts2).reshape(-1,1,2), 2,F)
        lines1 = lines1.reshape(-1,3)
        img5,_ = drawlines(img1,img2,lines1,np.int32(pts1),np.int32(pts2))
        lines2 = computeCorrespondEpilines(np.int32(pts1).reshape(-1,1,2), 1,F)
        lines2 = lines2.reshape(-1,3)
        img3,_ = drawlines(img2,img1,lines2,np.int32(pts2),np.int32(pts1))
        plt.figure(figsize=(20, 16),dpi=100)
        plt.subplot(121),plt.imshow(img5)
        plt.subplot(122),plt.imshow(img3)
        plt.show()
    return orderSecondFrame

class myServer(object):
    def __init__(self,numberCameras,triggerTime,recTime,FPS,verbose,output,save,ipList):
        ##########################################
        # PLEASE DONT CHANGE THE VARIABLES BELOW #
        ##########################################
        self.numberCameras,self.triggerTime,self.recTime,self.step,self.out,self.save,self.verbose,self.ipList = numberCameras,triggerTime,recTime,1/FPS,output,save,verbose,np.array(ipList.split(','))
        # check number of ips
        if self.ipList.shape[0]!=self.numberCameras:
            print('[ERROR] Number of cameras do not match the number of IPs given')
            exit()
        self.cameraMat,self.distCoef = np.copy(cameraMat),np.copy(distCoef)
        # do not change below this line, socket variables
        self.nImages,self.imageSize = int(self.recTime/self.step),[]
        for i in range(self.numberCameras): self.imageSize.append([])
        print('[INFO] creating server')
        self.bufferSize,self.server_socket = 1024,socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.server_socket.bind(('0.0.0.0',8888))

    # CONNECT WITH CLIENTS
    def connect(self):
        print("[INFO] server running, waiting for clients")
        addedCams,ports,k=[],[],0
        while k!=self.numberCameras:
            # COLLECT ADDRESSES
            message,address = self.server_socket.recvfrom(self.bufferSize)
            # CHECK IF IS IN IP LIST
            if not len(np.where(self.ipList==address[0])[0]):
                print('[ERROR] IP '+address[0]+' not in the list')
                exit()
            # GET IMAGE SIZE
            idx = np.where(self.ipList==address[0])[0][0]
            if idx in addedCams: continue
            self.imageSize[idx] = np.array(message.decode("utf-8").split(",")).astype(np.int)
            print('[INFO] camera '+str(idx)+' connected at '+str(address[0]))
            # REDO INTRINSICS
            ret,newCamMatrix=self.myIntrinsics(self.cameraMat[idx],self.imageSize[idx][0],self.imageSize[idx][1],self.imageSize[idx][2])
            if ret: self.cameraMat[idx]=np.copy(newCamMatrix)
            else: exit()
            k+=1
            addedCams.append(idx)
            ports.append(address)
        # SEND TRIGGER 
        print('[INFO] all clients connected')
        self.triggerTime += time.time()
        for i in range(self.numberCameras): self.server_socket.sendto((str(self.triggerTime)+' '+str(self.recTime)).encode(),tuple(ports[i]))
        print('[INFO] trigger sent')

    # NEW INTRINSICS
    def myIntrinsics(self,origMatrix,w,h,mode):
        camIntris = np.copy(origMatrix) # copy to avoid registrer error
        # check if image is at the vailable proportion
        if w/h==4/3 or w/h==16/9:
            if mode==4: # only resize
                ratio = w/960
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]
            elif mode==5: # crop in X and resize
                ratio = 1640/960
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]-155
                ratio = w/1640
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]
            elif mode==6: # crop in Y and X and resize
                ratio=1640/960
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]-180
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]-255
                ratio = w/1280
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]
            elif mode==7: # crop in Y and X and resize
                ratio=1640/960
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]-500
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]-375
                ratio = w/640
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]
            else:
                print('conversion for intrinsics matrix not known')
                return False,camIntris
            return True,camIntris
        else:
            print('out of proportion of the camera mode')
            return False,camIntris

    # COLLECT POINTS FROM CLIENTS, ORDER AND TRIGGER INTERPOLATION
    def collect(self):
        # internal variables
        print('[INFO] waiting capture')
        capture,counter = np.ones(self.numberCameras,dtype=np.bool),np.zeros(2,dtype=np.int8)
        dfSave,dfOrig = [],[]
        for i in range(self.numberCameras): dfOrig.append([])
        # capture loop
        try:
            while np.any(capture):
                #  receive message
                bytesPair = self.server_socket.recvfrom(self.bufferSize)
                message = np.frombuffer(bytesPair[0],dtype=np.float64)
                address,sizeMsg = bytesPair[1],len(message)
                idx = np.where(self.ipList==address[0])[0][0]
                if not (sizeMsg-1): capture[idx] = 0

                if capture[idx]: # check if message is valid
                    msg = message[0:sizeMsg-4].reshape(-1,3)
                    coord = msg[:,0:2]
                    # store message parameters
                    a,b,time,imgNumber = message[-4],message[-3],message[-2],int(message[-1]) 
                    if not len(coord): continue
                    # undistort points
                    undCoord = processCentroids(coord,a,b,self.cameraMat[idx],distCoef[idx])
                    if undCoord.shape[0]==3:
                        if self.save: dfSave.append(np.concatenate((undCoord.reshape(undCoord.shape[0]*undCoord.shape[1]),[time,imgNumber,idx])))
                        if not counter[idx]: dfOrig[idx] = np.hstack((undCoord.reshape(6),time))
                        counter[idx]+=1
                    # do I have enough points?
                    if np.all(counter[0:2]>0): break                                  
        finally:        
            # close everything
            self.server_socket.close()
            destroyAllWindows()
            # save results
            if self.save: np.savetxt('camGround.csv', np.array(dfSave), delimiter=',')
            # import R,T and lambda
            rotation = np.genfromtxt('R.csv', delimiter=',').reshape(-1,3,3)
            translation= np.genfromtxt('t.csv', delimiter=',').reshape(-1,1,3)
            scaleLamb,FMatrices = np.genfromtxt('lamb.csv', delimiter=','), np.genfromtxt('F.csv', delimiter=',').reshape(-1,3,3)
            # get matrices
            if rotation.shape[0]==2: R,t,lamb,F = rotation[1],translation[1],scaleLamb,FMatrices.reshape(3,3)
            else: R,t,lamb,F = rotation[1],translation[1],scaleLamb[0],FMatrices[0]
            # select points
            pts1,pts2 = dfOrig[0][0:6].reshape(-1,2),dfOrig[1][0:6].reshape(-1,2)
            orderSecondFrame = getOrderPerEpiline(pts1,pts2,3,np.copy(F))
            pts2 = np.copy(pts2[orderSecondFrame])
            pts1,pts2 = np.int32(pts1),np.int32(pts2)
            # plot epipolar lines
            img1,img2 = np.ones((540,960))*255,np.ones((540,960))*255
            lines1 = computeCorrespondEpilines(pts2.reshape(-1,1,2), 2,F)
            lines1 = lines1.reshape(-1,3)
            img5,_ = drawlines(img1,img2,lines1,pts1,pts2)
            lines2 = computeCorrespondEpilines(pts1.reshape(-1,1,2), 1,F)
            lines2 = lines2.reshape(-1,3)
            img3,_ = drawlines(img2,img1,lines2,pts2,pts1)
            plt.figure(figsize=(20, 16),dpi=100)
            plt.subplot(121),plt.imshow(img5)
            plt.subplot(122),plt.imshow(img3)
            plt.show()
            # triangulate
            P1,P2 = np.hstack((self.cameraMat[0], [[0.], [0.], [0.]])),np.matmul(self.cameraMat[1], np.hstack((R, t.T)))
            projPt1,projPt2 = myProjectionPoints(np.array(pts1)),myProjectionPoints(np.array(pts2))
            points4d = triangulatePoints(P1.astype(float),P2.astype(float),projPt1.astype(float),projPt2.astype(float))
            points3d = (points4d[:3, :]/points4d[3, :]).T
            if points3d[0, 2] < 0: points3d = -points3d
            plane = findPlane(points3d[0]*lamb/100,points3d[1]*lamb/100,points3d[2]*lamb/100)
            # compute plane
            [a,b,c,d]=plane
            v,k=np.array([a,b,c]),np.array([0,1,0])
            cosPhi = np.dot(v,k)/(np.linalg.norm(v)*np.linalg.norm(k))
            [u1,u2,u3] = np.cross(v,k)/np.linalg.norm(np.cross(v,k))
            sinPhi = np.sqrt(1-pow(cosPhi,2))
            R_plane = np.array([
                    [cosPhi+u1*u1*(1-cosPhi),u1*u2*(1-cosPhi),u2*sinPhi],
                    [u1*u2*(1-cosPhi),cosPhi+u2*u2*(1-cosPhi),-u1*sinPhi],
                    [-u2*sinPhi,u1*sinPhi,cosPhi]])
            [A,B,C] = np.matmul(np.array([a,b,c]),R_plane.T)
            newPlane = np.array([A,B,C])
            # plot
            points3dNew = points3d*lamb/100
            fig = plt.figure(figsize=(8, 8),dpi=100)
            ax = plt.axes(projection='3d')
            ax.set_xlim(-1, 4)
            ax.set_zlim(-5, 0)
            ax.set_ylim(-1, 4)
            ax.set_xlabel('X', fontweight='bold',labelpad=15)
            ax.set_ylabel('Z', fontweight='bold',labelpad=15)
            ax.set_zlabel('Y', fontweight='bold',labelpad=5)
            # plot first camera
            scale = 0.8
            cam1Pts = np.array([[0,d/b,0],
                                [scale, 0, 0],
                                [0, scale, 0],
                                [0, 0, scale]])
            cam1PtsNew = np.matmul(R_plane,cam1Pts.T).T
            [cam1Root,x,y,z]=cam1PtsNew
            zDisplacement = cam1PtsNew[0,2]
            cam1Root+=[0,0,-zDisplacement]
            x,y,z=x/np.linalg.norm(x)*scale,y/np.linalg.norm(y)*scale,z/np.linalg.norm(z)*scale
            ax.quiver(cam1Root[0],cam1Root[2],cam1Root[1],x[0],x[2],x[1], arrow_length_ratio=0.1, edgecolors="r", label = 'X axis')
            ax.quiver(cam1Root[0],cam1Root[2],cam1Root[1],y[0],y[2],y[1], arrow_length_ratio=0.1, edgecolors="b", label = 'Y axis')
            ax.quiver(cam1Root[0],cam1Root[2],cam1Root[1],z[0],z[2],z[1], arrow_length_ratio=0.1, edgecolors="g", label = 'Z axis')
            ax.scatter(cam1Root[0],cam1Root[2],cam1Root[1], s=50, edgecolor="fuchsia", facecolor="plum", linewidth=2, label = 'Camera 1')
            # plot second camera
            x,y,z = np.array([scale, 0, 0]), np.array([0, scale, 0]),np.array([0, 0, scale])
            x,y,z = np.matmul(R.T, x),np.matmul(R.T, y),np.matmul(R.T, z)
            t_aux = np.matmul(-t, R)[0]*lamb/100+[0,d/b,0]
            cam2Pts = np.array([t_aux.T,x,y,z])
            cam2PtsNew = np.matmul(R_plane,cam2Pts.T).T
            [cam2Root,x,y,z]=cam2PtsNew
            cam2Root+=[0,0,-zDisplacement]
            x,y,z=x/np.linalg.norm(x)*scale,y/np.linalg.norm(y)*scale,z/np.linalg.norm(z)*scale
            ax.quiver(cam2Root[0],cam2Root[2],cam2Root[1],x[0],x[2],x[1], arrow_length_ratio=0.1, edgecolors="r")
            ax.quiver(cam2Root[0],cam2Root[2],cam2Root[1],y[0],y[2],y[1], arrow_length_ratio=0.1, edgecolors="b")
            ax.quiver(cam2Root[0],cam2Root[2],cam2Root[1],z[0],z[2],z[1], arrow_length_ratio=0.1, edgecolors="g")
            ax.scatter(cam2Root[0],cam2Root[2],cam2Root[1], s=50, edgecolor="darkorange", facecolor="gold", linewidth=2,  label = 'Camera 2')
            # new plane
            x,z = np.linspace(-1,1,30),np.linspace(3,5,10)
            X,Z = np.meshgrid(x,z)
            Y=(-newPlane[0]*X -newPlane[2]*Z)/newPlane[1]
            surf = ax.plot_surface(X,Z-zDisplacement,Y,color='b',alpha=.15,label="Wand's plane")
            surf._facecolors2d = surf._facecolor3d
            surf._edgecolors2d = surf._edgecolor3d
            points3dNew = np.matmul(R_plane,points3dNew.T+np.array([[0],[d/b],[0]])).T
            ax.scatter(points3dNew[:, 0], points3dNew[:, 2]-zDisplacement, points3dNew[:, 1], color='black',s = 50,label='Wand 3D markers')
            ax.view_init(elev=30, azim=-120)
            plt.legend(ncol=3,loc ='center',edgecolor='silver', bbox_to_anchor=(0.5, 0.8)) 
            plt.gca().invert_zaxis()
            ax.get_proj = lambda: np.dot(Axes3D.get_proj(ax), np.diag([1., 1., .5, 1.]))
            #plt.savefig('withRot_2.png', bbox_inches='tight')
            plt.draw()
            plt.show()
 
            
# parser for command line
parser = argparse.ArgumentParser(description='''Server for the MoCap system at the Erobotica lab of UFCG.
                                                \nPlease use it together with the corresponding client script.''',add_help=False)
parser.add_argument('-n',type=int,help='number of cameras (default: 2)',default=2)
parser.add_argument('-trig',type=int,help='trigger time in seconds (default: 10)',default=10)
parser.add_argument('-rec',type=int,help='recording time in seconds (default: 30)',default=30)
parser.add_argument('-fps',type=int,help='interpolation fps (default: 100FPS)',default=100)
parser.add_argument('--verbose',help='show points capture after end of recording time (default: off)',default=False, action='store_true')
parser.add_argument('--help', action='help', default=argparse.SUPPRESS, help='show this help message and exit.')
parser.add_argument('-out', help='write the rotation matrix and translation vector(default: off)',default=False, action='store_true')
parser.add_argument('-save',help='save received packages to CSV (default: off)',default=False, action='store_true')
parser.add_argument('-ip',type=str,help='List of IPs of each camera, in order',default='')
args = parser.parse_args()

myServer_ = myServer(args.n,args.trig,args.rec,args.fps,args.verbose,args.out,args.save,args.ip)
myServer_.connect()
myServer_.collect()