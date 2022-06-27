# IMPORTS >>> DO NOT CHANGE <<<
import warnings
warnings.filterwarnings("ignore")
import socket,time,argparse
import numpy as np
from cv2 import destroyAllWindows,triangulatePoints,computeCorrespondEpilines
from myLib import myProjectionPoints,processCentroids,isEqual4,getOrderPerEpiline,findPlane,drawlines
from constants import cameraMat,distCoef
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
        capture,counter = np.ones(self.numberCameras,dtype=np.bool),np.zeros(self.numberCameras,dtype=np.int8)
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
            scale,FMatrices = np.genfromtxt('lamb.csv', delimiter=','), np.genfromtxt('F.csv', delimiter=',').reshape(-1,3,3)
            # get matrices
            if rotation.shape[0]==2: R,t,lamb,F = rotation[1],translation[1],scale[1],FMatrices.reshape(3,3)
            else: R,t,lamb,F = rotation[1],translation[1],scale[1],FMatrices[0]
            # select points
            pts1,pts2 = dfOrig[0][0:6].reshape(-1,2),dfOrig[1][0:6].reshape(-1,2)
            orderSecondFrame = getOrderPerEpiline(pts1,pts2,3,np.copy(F))
            pts2 = np.copy(pts2[orderSecondFrame])
            pts1,pts2 = np.int32(pts1),np.int32(pts2)
            # plot epipolar lines
            '''img1,img2 = np.ones((720,960))*255,np.ones((720,960))*255
            lines1 = computeCorrespondEpilines(pts2.reshape(-1,1,2), 2,F)
            lines1 = lines1.reshape(-1,3)
            img5,_ = drawlines(img1,img2,lines1,pts1,pts2)
            lines2 = computeCorrespondEpilines(pts1.reshape(-1,1,2), 1,F)
            lines2 = lines2.reshape(-1,3)
            img3,_ = drawlines(img2,img1,lines2,pts2,pts1)
            plt.figure(figsize=(20, 16),dpi=100)
            plt.subplot(121),plt.imshow(img5)
            plt.subplot(122),plt.imshow(img3)
            plt.show()'''
            # triangulate ordered centroids
            P1,P2 = np.hstack((cameraMat[0], [[0.], [0.], [0.]])),np.matmul(cameraMat[1], np.hstack((R, t.T)))
            projPt1,projPt2 = myProjectionPoints(np.array(pts1)),myProjectionPoints(np.array(pts2))
            points4d = triangulatePoints(P1.astype(float),P2.astype(float),projPt1.astype(float),projPt2.astype(float))
            points3d = (points4d[:3, :]/points4d[3, :]).T
            if points3d[0, 2] < 0: points3d = -points3d
            points3d = points3d*lamb/100
            # get ground plane coefficients
            plane = findPlane(points3d[0],points3d[1],points3d[2])
            if np.any(plane[0:3]<0): plane = findPlane(points3d[0],points3d[2],points3d[1])
            [a,b,c,d]=plane
            # get the orthonogal vector to the plane
            v,k=np.array([a,b,c]),np.array([0,1,0])
            # compute the angle between the plane and the y axis
            cosPhi = np.dot(v,k)/(np.linalg.norm(v)*np.linalg.norm(k))
            # compute the versors
            [u1,u2,_] = np.cross(v,k)/np.linalg.norm(np.cross(v,k))
            # get the rotation matrix and new ground plane coefficients
            sinPhi = np.sqrt(1-pow(cosPhi,2))
            R_plane = np.array([
                    [cosPhi+u1*u1*(1-cosPhi),u1*u2*(1-cosPhi),u2*sinPhi],
                    [u1*u2*(1-cosPhi),cosPhi+u2*u2*(1-cosPhi),-u1*sinPhi],
                    [-u2*sinPhi,u1*sinPhi,cosPhi]])
            [A,B,C] = np.matmul(np.array([a,b,c]),R_plane.T)
            newPlane = np.array([A,B,C])
            
            # setting 3D plot variables
            fig = plt.figure(figsize=(8, 8),dpi=100)
            ax = plt.axes(projection='3d')
            ax.set_xlim(-1, 7)
            ax.set_zlim(-6, 0)
            ax.set_ylim(-1, 7)
            ax.set_xlabel('X')
            ax.set_ylabel('Z')
            ax.set_zlabel('Y')
            ax.set_xlabel('X', fontweight='bold',labelpad=15)
            ax.set_ylabel('Z', fontweight='bold',labelpad=15)
            ax.set_zlabel('Y', fontweight='bold',labelpad=5)
            cmhot = plt.get_cmap("jet")
            ax.view_init(elev=30, azim=-50) 
            plt.gca().invert_zaxis()
            ax.get_proj = lambda: np.dot(Axes3D.get_proj(ax), np.diag([1., 1., .5, 1.]))
            colours = [['fuchsia','plum'],['darkorange','gold'],['limegreen','greenyellow'],['blue','lightsteelblue']]
            # initializing arrays
            zDisplacement = 0
            P_plane = np.vstack((np.hstack((R_plane,np.array([0,0,0]).reshape(3,-1))),np.hstack((np.zeros((3)),1))))
            # plot each camera translated and rotated to meet the ground plane
            for j in range(self.numberCameras):
                # initialize initial values of the projection matrices                
                P_new = np.vstack((np.hstack((np.identity(3),np.zeros((3,1)))),np.hstack((np.zeros((3)),1))))
                # iterate over the previous cameras to find the relation to the 0th camera
                # e.g. 3 -> 2 -> 1 -> 0
                for i in np.flip(range(j+1)):
                    t,R,lamb = np.array(translation[i][0]).reshape(-1,3),np.array(rotation[i]),scale[i]
                    t_new = np.matmul(-t, R).reshape(-1,3)*lamb/100
                    P = np.vstack((np.hstack((R.T,t_new.T)),np.hstack((np.zeros((3)),1))))
                    P_new = np.matmul(P,P_new)
                o = np.matmul(P_new,[[0.],[0],[0.],[1]]).ravel()
                o+= [0,+d/b,0,0]
                o = np.matmul(P_plane,o).ravel()
                if not j: zDisplacement = o[2]
                o+= [0,0,-zDisplacement,0]
                x,y,z= np.array([1, 0, 0, 0]), np.array([0, 1, 0, 0]),np.array([0, 0, 1, 0])
                x,y,z = np.matmul(P_new,x),np.matmul(P_new,y),np.matmul(P_new,z)
                x,y,z = np.matmul(P_plane,x),np.matmul(P_plane,y),np.matmul(P_plane,z)
                ax.quiver(o[0], o[2], o[1], x[0], x[2], x[1], arrow_length_ratio=0.1, edgecolors="r", label='X axis')
                ax.quiver(o[0], o[2], o[1], y[0], y[2], y[1], arrow_length_ratio=0.1, edgecolors="b", label='Y axis')
                ax.quiver(o[0], o[2], o[1], z[0], z[2], z[1], arrow_length_ratio=0.1, edgecolors="g", label='Z axis')
                ax.scatter(o[0], o[2], o[1], s=50, edgecolor=colours[j][0], facecolor=colours[j][1], linewidth=2,  label = 'Camera '+str(j))                
            # plot the ground plane
            x,z = np.linspace(-1,1,30),np.linspace(3,5,10)
            X,Z = np.meshgrid(x,z)
            Y=(-newPlane[0]*X -newPlane[2]*Z)/newPlane[1]
            surf = ax.plot_surface(X,Z-zDisplacement,Y,color='b',alpha=.15,label="Wand's plane")
            surf._facecolors2d = surf._facecolor3d
            surf._edgecolors2d = surf._edgecolor3d
            # plot markers
            points3d = np.hstack((points3d,np.ones((points3d.shape[0],1))))
            points3d+= [0,d/b,0,0]
            points3d = np.matmul(P_plane,points3d.T).T
            points3d+= [0,0,-zDisplacement,0]
            points3d = points3d.T
            ax.scatter(points3d[0], points3d[2], points3d[1], s=50, c=points3d[2], cmap=cmhot, label= 'Markers')
            # axis setup and plot variables
            handles, labels = plt.gca().get_legend_handles_labels()
            by_label = dict(zip(labels, handles))
            plt.legend(by_label.values(), by_label.keys(),ncol=3,loc ='center',edgecolor='silver', bbox_to_anchor=(0.5, 0.8))            
            # saving csv
            np.savetxt('groundCalibData.csv', np.hstack(([zDisplacement,d/b],R_plane.ravel())), delimiter=',')
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