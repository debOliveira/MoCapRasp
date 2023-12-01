import numpy as np

# Intrinsics Matrices
intrinsicsMatrix_0 = np.array([[720.313,0,481.014],
                               [0,719.521,360.991],
                               [0,0,1]])
distortionCoefficients_0 = np.array([[0.395621],
                                     [0.633705],
                                     [-2.41723],
                                     [2.11079]], dtype=np.float32)

intrinsicsMatrix_1 = np.array([[768.113,0,472.596],
                               [0,767.935,350.978],
                               [0,0,1]])
distortionCoefficients_1 = np.array([[0.368917],
                                     [1.50111],
                                     [-7.94126],
                                     [11.9171]], dtype=np.float32)

intrinsicsMatrix_2 = np.array([[728.237,0,459.854],
                               [0,729.419,351.59],
                               [0,0,1]])
distortionCoefficients_2 = np.array([[0.276114],
                                     [2.09465],
                                     [-9.97956],
                                     [14.1921]], dtype=np.float32)

intrinsicsMatrix_3 = np.array([[750.149,0,492.144],
                               [0,748.903,350.213],
                               [0,0,1]])
distortionCoefficients_3 = np.array([[0.400774],
                                     [1.15995],
                                     [-7.10257],
                                     [11.415]], dtype=np.float32)

# All in One
intrinsicsMatrix = [intrinsicsMatrix_0,
                    intrinsicsMatrix_1,
                    intrinsicsMatrix_2,
                    intrinsicsMatrix_3]
distortionCoefficients = [distortionCoefficients_0,
                          distortionCoefficients_1,
                          distortionCoefficients_2,
                          distortionCoefficients_3]