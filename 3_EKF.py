##
# Main function of the Python program.
#
##

import pandas as pd
import numpy as np
from numpy.linalg import inv
import math


#Fill in Those functions:

def computeRmse(trueVector, EstimateVector):
    list(trueVector)
    list(EstimateVector)
    length = len(EstimateVector[0])
    rmse = []
    for i in range(len(EstimateVector)):
        rmse.append(np.sqrt(1/length * ((EstimateVector[i] - trueVector[i]) ** 2)))
    return rmse     
def computeFmatrix(deltaT):
    F = np.array([
         [1, 0, deltaT, 0],
         [0, 1, 0, deltaT],
         [0, 0, 1, 0],
         [0, 0, 0, 1]])
    return F     
    
def main():
    # we print a heading and make it bigger using HTML formatting
    print("Hellow")
    my_cols = ["0", "1", "2", "3", "4","5","6","7","8","9","10"]
    data = pd.read_csv("C:/Users/tomer/Desktop/obj_pose-laser-radar-synthetic-input.txt", names=my_cols, delim_whitespace = True, header=None)
    print(data.head())
    for i in range(10):
        measur = data.iloc[i,:].values
        print(measur[0])
    # define matrices:
    deltaT = 1.0
    useRadar = False
    # fill diagonal 12 to p matrix
    P = np.array([
        [12, 0, 0, 0],
        [0, 12, 0, 0],
        [0, 0, 12, 0],
        [0, 0, 0, 12]
        ])
    xEstimate = []
    xTrue = []
    H_Lidar = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0]])
    R_lidar = np.array([
                    [0.0225, 0.0],
                    [0.0, 0.0225]
                    ])
    F_matrix = computeFmatrix(deltaT)
    X_state_current = np.array([[5], 
                                [5],
                                [0],
                                [0]])
    X_true_current = np.zeros([4,1])
    I = np.identity(4)
    z = np.zeros([2,1])
    firstMeasurment = data.iloc[0,:].values
    timeStamp = firstMeasurment[3]
    #fill in X_true and X_state. Put 0 for the velocities
    for index in range(len(data)):
        currentMeas = data.iloc[index,:].values
        if(currentMeas[0]=='L'):
            # compute the current dela t          
            deltaT = (currentMeas[3]- timeStamp)/1000000
            # prev_time
            timeStamp = currentMeas[3]
            #Updating sensor readings
            z[0][0] = currentMeas[1]
            z[1][0] = currentMeas[2]
            #collecting ground truth
            X_true_current[0] = currentMeas[4]
            X_true_current[1] = currentMeas[5]
            X_true_current[2] = currentMeas[6]
            X_true_current[3] = currentMeas[7]

            #perfrom predict
            # X*F
            X_state_current = np.matmul(F_matrix,X_state_current)
            #  P = F * P * F(transpse) + Q| Q = 0;
            Ft = np.transpose(F_matrix)
            P  = np.matmul(F_matrix, np.matmul(P, Ft))

            #pefrom measurment update
            y = np.subtract(z, np.matmul(H_Lidar, X_state_current))
            H_Lidar_T = np.transpose(H_Lidar)
            S = np.add(np.matmul(H_Lidar, np.matmul(P, H_Lidar_T)), R_lidar)
            K = np.matmul(P, H_Lidar_T)
            #inverse S
            SI = inv(S)
            K = np.matmul(K ,SI)
            X_state_current = np.add(X_state_current, np.matmul(K, y))
            P  = np.matmul(np.subtract(I, np.matmul(K, H_Lidar)), P)

        # if(currentMeas[0]=='R' and useRadar):
        #      #perfrom predict
        #     deltaT = (currentMeas[4]- timeStamp)/1000000
        #     timeStamp = currentMeas[4]
        #     X_state_current = 0
        #     P  =  P
            
        #     #pefrom measurment update
        #     jacobian = 0
        #     z = 0
        #     S = 0
        #     K = 0
        #     X_state_current = 0 
        #     P  = 0
        xEstimate.append(X_state_current)
        xTrue.append(X_true_current)
    rmse = computeRmse(xEstimate, xTrue) 
    print(rmse)
if __name__ == '__main__':
    main()
