import numpy as np

# camera matrices
cameraMatrix_cam1 =np.array([[720.313,0,481.014],[0,719.521,360.991],[0,0,1]])
distCoef_cam1 = np.array([[0.395621],[0.633705],[-2.41723],[2.11079]], dtype=np.float32)

cameraMatrix_cam2 =np.array([[768.113,0.,472.596],[0.,767.935,350.978],[0,0,1]])
distCoef_cam2 = np.array([[0.368917],[1.50111],[-7.94126],[11.9171]], dtype=np.float32)

cameraMatrix_cam3 =np.array([[728.237,0,459.854],[0,729.419,351.59],[0,0,1]])
distCoef_cam3 = np.array([[0.276114],[2.09465],[-9.97956],[14.1921]], dtype=np.float32)

cameraMatrix_cam4 =np.array([[750.149,0,492.144],[0,748.903,350.213],[0,0,1]])
distCoef_cam4 = np.array([[0.400774],[1.15995],[-7.10257],[11.415]], dtype=np.float32)

# all in one
cameraMat = [cameraMatrix_cam1,cameraMatrix_cam2,cameraMatrix_cam3,cameraMatrix_cam4]
distCoef = [distCoef_cam1,distCoef_cam2,distCoef_cam3,distCoef_cam4]