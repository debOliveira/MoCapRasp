import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# PLOTTING

def plotArena(title, allPoints3d, cameraData = None, groundData = None):
    # Setting 3D plot variables
    fig = plt.figure(figsize=(8, 8),dpi=100)
    ax = plt.axes(projection='3d')
    ax.set_xlim(-1, 4)
    ax.set_zlim(-6, 0)
    ax.set_ylim(-1, 4)
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
    
    if cameraData is not None:
        cameraHeight = cameraData.get('cameraHeight')
        projectionMatrices = cameraData.get('projectionMatrices')

    # Get ground data
    if groundData is not None:
        planeDisplacement = groundData.get('planeDisplacement')
        planeRotation = groundData.get('planeRotation')
        planeCoefficients = groundData.get('planeCoefficients')

    # Plot cameras
    if cameraData is not None:
        for cam in range(len(projectionMatrices)):
            # Plot camera pose
            x,y,z = np.array([1, 0, 0, 0]), np.array([0, 1, 0, 0]),np.array([0, 0, 1, 0])
            x,y,z = np.matmul(projectionMatrices[cam],x),np.matmul(projectionMatrices[cam],y),np.matmul(projectionMatrices[cam],z)

            # Adjust camera location
            o = np.matmul(projectionMatrices[cam],[[0.],[0.],[0.],[1]]).ravel()

            # If ground data is given
            if groundData is not None:
                o += [0,planeDisplacement,0,0] # Displace in relation to ground
                o = np.matmul(planeRotation,o).ravel() # Rotate in relation to ground
                o += [0,0,-cameraHeight,0] # Displacing each camera

                # Rotate in relation to ground
                x,y,z = np.matmul(planeRotation,x),np.matmul(planeRotation,y),np.matmul(planeRotation,z)

            # Plot camera pose
            ax.quiver(o[0], o[2], o[1], x[0], x[2], x[1], arrow_length_ratio=0.05, edgecolors="r", label='X axis')
            ax.quiver(o[0], o[2], o[1], y[0], y[2], y[1], arrow_length_ratio=0.05, edgecolors="b", label='Y axis')
            ax.quiver(o[0], o[2], o[1], z[0], z[2], z[1], arrow_length_ratio=0.05, edgecolors="g", label='Z axis')

            # Plot camera position
            ax.scatter(o[0], o[2], o[1], s=50, edgecolor=colours[cam][0], facecolor=colours[cam][1], linewidth=2,  label = 'Camera '+str(cam))
 
    # Plot the ground plane if possible
    if groundData is not None:
        if planeCoefficients is not None:
            x,z = np.linspace(-1,1,30),np.linspace(3,5,10)
            X,Z = np.meshgrid(x,z)
            Y = (-planeCoefficients[0]*X -planeCoefficients[2]*Z)/planeCoefficients[1]

            print('[RESULTS] Maximum Y:',max(np.array(Y).ravel()))
            print('[RESULTS] Minimum Y:',min(np.array(Y).ravel()))

            # Plot surface
            surf = ax.plot_surface(X,Z-cameraHeight,Y,color='b',alpha=.15,label="Wand's plane")
            surf._facecolors2d = surf._facecolor3d
            surf._edgecolors2d = surf._edgecolor3d

    # Plot 3D Points
    ax.scatter(allPoints3d[0], allPoints3d[2], allPoints3d[1], s=50, c=allPoints3d[2], cmap=cmhot)
        
    # Final plot commands
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(),ncol=3,loc ='center',edgecolor='silver', bbox_to_anchor=(0.5, 0.8))
    plt.title(title)
    plt.draw()
    plt.show()