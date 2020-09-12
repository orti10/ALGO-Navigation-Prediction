from numpy import *
from numpy.linalg import inv
import numpy as np 
# X : The mean state estimate of the previous step ( k −1).
# P : The state covariance of previous step ( k −1).
# F : The transition n n × matrix.
# K : the Kalman Gain matrix
# IM : the Mean of predictive distribution of Y
# IS : the Covariance or predictive mean of Y
def Kalman_Measurement_Update(H,F,P,X,R,Z):
   IM = dot(H, X)
   IS = dot(H, dot(P, H.T))
   K = dot(P, dot(np.transpose(H), inv(IS)))
   X = X + dot(K, (Z-IM))
   P = P - dot(K, dot(IS, np.transpose(K)))
   return X, P
     
