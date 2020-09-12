##
# Main function 
#
##
import numpy as np
from numpy.linalg import inv
import KalmanFilter as KF
# measurement update
P = np.array([[45,0],[0,25]])
F = np.array([[1,1],[0,1]])
H = np.array([[1,0]])
R = np.array([[0,0],[0,0]])
X =  np.array([[4281],[282]])
Z = np.array([[4260,282]])

XK,PK = KF.Kalman_Measurement_Update(H,F,P,X,R,Z)
print("XK : \n", XK)
print ("PK : \n" , PK)