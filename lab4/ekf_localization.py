#!/usr/bin/python
# -*- coding: utf-8 -*-

"""EKF class that Implements prediction and update."""

import numpy as np
import probabilistic_lib.functions as funcs
# use: comp, get_polar_line, get_map


# ==============================================================================
class EKF(object):
    """Class to hold the whole Extended Kalman Filter (EKF)."""

    # ==========================================================================
    def __init__(self, xinit, odom_lin_sigma, odom_ang_sigma, meas_rng_noise,
                 meas_ang_noise):
        """
        Initialize the EKF filter.

        Input:
          room_map : a nx4 array of lines in the form [x1 y1 x2 y2]
          xinit    : initial position
          odom_lin_sigma: odometry linear noise
          odom_ang_sigma: odometry angular noise
          meas_rng_noise: measurement linear noise
          meas_ang_noise: measurement angular noise
        """
        # Map with initial displacement
        self.map = funcs.get_dataset3_map(xinit[0], xinit[1], xinit[2])

        # Prediction noise
        self.Qk = np.array([[odom_lin_sigma**2, 0, 0],
                            [0, odom_lin_sigma**2, 0],
                            [0, 0, odom_ang_sigma**2]])

        # Measurement noise
        self.Rk = np.array([[meas_rng_noise**2, 0],
                            [0, meas_ang_noise**2]])

        # Pose initialization
        self.xk = np.zeros(3)
        self.Pk = 0.2*0.2*np.eye(3)  # initial uncertainty of robot state

    # ==========================================================================
    def predict(self, uk):
        """
        Implement the prediction equations of the EKF.

        Saves the new robot state and uncertainty.

        Input:
          uk : numpy array [shape (3,) with (dx, dy, dtheta)]
        """
        # TODO: Program this function
        ################################################################
        # Check website for numpy help:
        #         http://wiki.scipy.org/NumPy_for_Matlab_Users
        # 1. Update self.xk and self.Pk using uk and self.Qk
        #        can use comp() from funtions.py
        
        # Calculate new state vector components
        c = np.cos(self.xk[2])
        s = np.sin(self.xk[2])
        xr = self.xk[0] + uk[0]*c - uk[1]*s
        yr = self.xk[1] + uk[0]*s + uk[1]*c
        theta = funcs.angle_wrap(self.xk[2] + uk[2])
        self.xk = np.array([xr,yr,theta])# Should this be here?

        # Calculate the current Ak matrix
        A13 = - uk[0]*s - uk[1]*c
        A23 = uk[0]*c - uk[1]*s
        Ak = np.array([[1,0,A13],[0,1,A23],[0,0,1]])
        # print(Ak)

        # Calculate Wk
        Wk = np.array([[c,-s,0],[s,c,0],[0,0,1]])

        # Calculate Pk'
        self.Pk = np.dot(Ak,np.dot(self.Pk,Ak.T)) + np.dot(Wk,np.dot(self.Qk,Wk.T))



    # ==========================================================================
    def data_association(self, lines):
        """
        Look for closest correspondences.

        The correspondences are between the provided measured lines and the map
        known a priori.

        Input:
          lines : nx4 matrix with a segment in each row as [x1 y1 x2 y2]
        Return:
          Hk_list : list of 2x3 matrices (jacobian)
          Yk_list : list of 2x1 matrices (innovation)
          Sk_list : list of 2x2 matrices (innovation uncertainty)
          Rk_list : list of 2x2 matrices (measurement uncertainty)
        """
        # TODO: Program this function
        ################################################################
        # 1. Map lines (self.map) to polar robot frame: get_polar_line
        # 2. Sensed lines (lines) to polar robot frame: get_polar_line
        # 3. Data association

        # Init variables
        # chi_thres = 0.103  # chi square 2DoF 95% confidence
        chi_thres = 0.3
        associd = list()
        Hk_list = list()
        Vk_list = list()
        Sk_list = list()
        Rk_list = list()

        # return Hk_list, Vk_list, Sk_list, Rk_list  # TODO: delete this line

        # For each obseved line
        # print('\n-------- Associations --------')
        for i in range(0, lines.shape[0]):

            # Get the polar line representation in robot frame
            z = funcs.get_polar_line(lines[i,:])
            length_z = np.sqrt((lines[i][0]-lines[i][2])**2 + (lines[i][1]-lines[i][3])**2)

            # Variables for finding minimum
            minD = 1e9
            minj = -1

            # For each line in the known map
            for j in range(0, self.map.shape[0]):

                # Compute matrices
                h = funcs.get_polar_line(self.map[j,:],self.xk)
                H = self.jacobianH(funcs.get_polar_line(self.map[j,:]),self.xk)
                v = z - h
                S = np.dot(H,np.dot(self.Pk,H.T)) + self.Rk

                # Mahalanobis distance
                D = np.dot(v.T,np.dot(np.linalg.inv(S),v))

                # Optional: Check if observed line is longer than map
                ########################################################
                length_h = np.sqrt((self.map[j][0]-self.map[j][2])**2 + (self.map[j][1]-self.map[j][3])**2)
                if length_z > (1.1*length_h):
                    islonger = True
                else:
                    islonger = False

                # Check if the obseved line is the one with smallest
                # mahalanobis distance
                if np.sqrt(D) < minD and not islonger:
                    minj = j
                    minz = z
                    minh = h
                    minH = H
                    minv = v
                    minS = S
                    minD = D

            # Minimum distance below threshold
            if minD < chi_thres:
                # print("\t{:.2f} -> {:.2f}".format(minz, minh))
                # Append results
                associd.append([i, minj])
                Hk_list.append(minH)
                Vk_list.append(minv)
                Sk_list.append(minS)
                Rk_list.append(self.Rk)

        return Hk_list, Vk_list, Sk_list, Rk_list

    # ==========================================================================
    def update_position(self, Hk_list, Vk_list, Sk_list, Rk_list):
        """
        Update the position of the robot according to the given matrices.

        The matrices contain the current position and the data association
        parameters. All input lists have the same lenght.

        Input:
          Hk_list : list of 2x3 matrices (jacobian)
          Yk_list : list of 2x1 matrices (innovation)
          Sk_list : list of 2x2 matrices (innovation uncertainty)
          Rk_list : list of 2x2 matrices (measurement uncertainty)
        """
        # Compose list of matrices as single matrices
        n = len(Hk_list)
        H = np.zeros((2*n, 3))
        v = np.zeros((2*n))
        S = np.zeros((2*n, 2*n))
        R = np.zeros((2*n, 2*n))
        for i in range(n):
            H[2*i:2*i+2, :] = Hk_list[i]
            v[2*i:2*i+2] = Vk_list[i]
            S[2*i:2*i+2, 2*i:2*i+2] = Sk_list[i]
            R[2*i:2*i+2, 2*i:2*i+2] = Rk_list[i]

        # There is data to update
        if not n > 0:
            return

        # TODO: Program this function
        ################################################################
        # Do the EKF update
        K = np.dot(self.Pk,np.dot(H.T,np.linalg.inv(S)))
        self.xk = self.xk + np.dot(K,v)
        aux = np.eye(3) - np.dot(K,H)
        self.Pk = np.dot(aux,np.dot(self.Pk,aux.T)) + np.dot(K,np.dot(R,K.T))

        
    # ==========================================================================
    def jacobianH(self, lineworld, xk):
        """
        Compute the jacobian of the get_polar_line function.

        It does it with respect to the robot state xk (done in pre-lab).
        """
        # TODO: Program this function
        ################################################################
        # Complete the Jacobian H from the pre-lab
        # Jacobian H

        c = np.cos(lineworld[1])
        s = np.sin(lineworld[1])
        rho_prime = lineworld[0] - xk[0]*c - xk[1]*s

        if rho_prime >= 0:
            H = np.array([[-c,-s,0],[0,0,-1]])
        else:
            H = np.array([[c,s,0],[0,0,-1]])
        # print(H)

        return H
