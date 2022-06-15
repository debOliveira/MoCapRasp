# IMPORTS >>> DO NOT CHANGE <<<
import warnings
warnings.filterwarnings("ignore")
import socket,time,math,argparse
import numpy as np
from functions import processCentroids_calib,cameraMatrix_cam1,cameraMatrix_cam2,distCoef_cam1,distCoef_cam2
from cv2 import circle,COLOR_GRAY2RGB,destroyAllWindows,triangulatePoints,cvtColor,line,COLOR_GRAY2BGR,computeCorrespondEpilines
from myLib import orderCenterCoord,estimateFundMatrix_8norm,decomposeEssentialMat,myProjectionPoints,isCollinear,isEqual,getSignal
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline

def isEqual4(pt,tol=5):
    A,B,C,D = pt[0],pt[1],pt[2],pt[3]
    AB,AC,BC = np.linalg.norm(A-B),np.linalg.norm(A-C),np.linalg.norm(C-B)
    AD,BD,CD = np.linalg.norm(A-D),np.linalg.norm(B-D),np.linalg.norm(C-D)
    return min(AB,AC,BC,AD,BD,CD)<tol

def getEpilineCoef(pts,F):
    [a,b,c]=np.matmul(F,np.hstack((pts,1))) #ax+by+c=0
    return [a,b,c]/(np.sqrt(pow(a,2)+pow(b,2)))

def getDistance2Line(line,pts):
    a,b,c = line
    out = []
    for [x,y] in pts:
        out.append(abs(a*x+b*y+c)/np.sqrt(np.sqrt(pow(a,2)+pow(b,2))))
    return np.array(out)    

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

class myServer(object):
    def __init__(self,numberCameras,triggerTime,recTime,FPS,verbose,output,save,firstIP):
        ##########################################
        # PLEASE CHANGE JUST THE VARIABLES BELOW #
        ##########################################
        self.numberCameras,self.triggerTime,self.recTime,self.step,self.out,self.save,self.verbose,self.firstIP = numberCameras,triggerTime,recTime,1/FPS,output,save,verbose,firstIP
        self.cameraMat = np.array([cameraMatrix_cam1,cameraMatrix_cam2])
        # do not change below this line, socket variables
        self.nImages,self.imageSize = int(self.recTime/self.step),[]
        print('[INFO] creating server')
        self.bufferSize,self.server_socket = 1024,socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.server_socket.bind(('0.0.0.0',8888))
        # internal variables
        self.addDic,self.myIPs= {},[]

    # CONNECT WITH CLIENTS
    def connect(self):
        print("[INFO] server running, waiting for clients")
        for i in range(self.numberCameras):
            # COLLECT ADDRESSES
            message,address = self.server_socket.recvfrom(self.bufferSize)
            self.imageSize.append(np.array(message.decode("utf-8").split(",")).astype(np.int))
            self.addDic[address[0]]=i
            self.myIPs.append(address)
            print('[INFO] client '+str(len(self.addDic))+' connected at '+str(address[0]))
            ret,newCamMatrix=self.myIntrinsics(self.cameraMat[i],self.imageSize[i][0],self.imageSize[i][1],self.imageSize[i][2])
            if ret: self.cameraMat[i]=np.copy(newCamMatrix)
            else: break
        # SEND TRIGGER 
        print('[INFO] all clients connected')
        self.triggerTime += time.time()
        for i in range(self.numberCameras): self.server_socket.sendto((str(self.triggerTime)+' '+str(self.recTime)).encode(),tuple(self.myIPs[i]))
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
            print('[INFO] intrisics known')
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
                idx = self.addDic[address[0]]
                if not (sizeMsg-1): capture[idx] = 0

                if capture[idx]: # check if message is valid
                    msg = message[0:sizeMsg-4].reshape(-1,3)
                    coord = msg[:,0:2]
                    # store message parameters
                    a,b,time,imgNumber = message[-4],message[-3],message[-2],int(message[-1]) 
                    if not len(coord): continue
                    # undistort points
                    if address[0] == self.firstIP: 
                        print('in')
                        undCoord = processCentroids_calib(coord,a,b,self.cameraMat[0],distCoef_cam1)
                    else: undCoord = processCentroids_calib(coord,a,b,self.cameraMat[1],distCoef_cam2)     
                    if undCoord.shape[0]==3:
                        if self.save: dfSave.append(np.concatenate((undCoord.reshape(undCoord.shape[0]*undCoord.shape[1]),[time,imgNumber,idx])))
                        if not counter[idx]: dfOrig[idx] = np.hstack((undCoord.reshape(6),time))
                        counter[idx]+=1
                    # do I have enough points?
                    if np.all(counter>0): break                                            
        finally:        
            # close everything
            self.server_socket.close()
            destroyAllWindows()
            # save results
            if self.save: np.savetxt('camGround.csv', np.array(dfSave), delimiter=',')
            # import R,T and lambda
            R,t,lamb,F = np.genfromtxt('R.csv', delimiter=','),np.genfromtxt('t.csv', delimiter=',').reshape(-1,3),np.genfromtxt('lamb.csv', delimiter=','), np.genfromtxt('F.csv', delimiter=',')
            print(t)
            # select points
            pts1,pts2,orderSecondFrame = np.int32(dfOrig[0][0:6].reshape(-1,2)),np.int32(dfOrig[1][0:6].reshape(-1,2)),[]
            for i in range(3):
                epiline = getEpilineCoef(pts1[i],F)
                orderSecondFrame.append(np.argmin(getDistance2Line(epiline,pts2)))
            pts2 = np.copy(pts2[orderSecondFrame])
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
parser.add_argument('-ip',type=str,help='IP from first camera (default: 192.168.0.102)',default='192.168.0.102')
args = parser.parse_args()

myServer_ = myServer(args.n,args.trig,args.rec,args.fps,args.verbose,args.out,args.save,args.ip)
myServer_.connect()
myServer_.collect()