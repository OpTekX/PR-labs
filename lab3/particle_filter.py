#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
from probabilistic_lib.functions import angle_wrap, get_polar_line

#===============================================================================
class ParticleFilter(object):
    '''
    Class to hold the whole particle filter.
    
    p_wei: weights of particles in array of shape (N,)
    p_ang: angle in radians of each particle with respect of world axis, shape (N,)
    p_xy : position in the world frame of the particles, shape (2,N)
    '''
    
    #===========================================================================
    def __init__(self, room_map, num, odom_lin_sigma, odom_ang_sigma, 
                 meas_rng_noise, meas_ang_noise,x_init,y_init,theta_init):
        '''
        Initializes the particle filter
        room_map : an array of lines in the form [x1 y1 x2 y2]
        num      : number of particles to use
        odom_lin_sigma: odometry linear noise
        odom_ang_sigma: odometry angular noise
        meas_rng_noise: measurement linear noise
        meas_ang_noise: measurement angular noise
        '''
        
        # Copy parameters
        self.map = room_map
        self.num = num
        self.odom_lin_sigma = odom_lin_sigma
        self.odom_ang_sigma = odom_ang_sigma
        self.meas_rng_noise = meas_rng_noise
        self.meas_ang_noise = meas_ang_noise
        
        # Map
        map_xmin = np.min(self.map[:, 0])
        map_xmax = np.max(self.map[:, 0])
        map_ymin = np.min(self.map[:, 1])
        map_ymax = np.max(self.map[:, 1])
        
        # Particle initialization arround starting point
        self.p_wei = 1.0 / num * np.ones(num)
        self.p_ang =2 * np.pi * np.random.rand(num)
        self.p_xy  = np.vstack(( x_init+ 1*np.random.rand(num)-0.5,
                                 y_init+ 1*np.random.rand(num)-0.5 ))
        #Flags for resampling                         
        self.moving=False
        self.n_eff=0 #Initialize Efficent number as 0

    
    #===========================================================================
    def predict(self, odom):
        '''
        Moves particles with the given odometry.
        odom: incremental odometry [delta_x delta_y delta_yaw] in the vehicle frame
        '''
        #Check if we have moved from previous reading.
        if odom[0]==0 and odom[1]==0 and odom[2]==0:
            self.moving=False
        else:
            # TODO: code here!!
            # Add Gaussian noise to odometry measures
            xy_noise = np.random.randn(2,self.num) * self.odom_lin_sigma
            ang_noise = np.random.randn(self.num) * self.odom_ang_sigma
            
            
            # Increment particle positions in correct frame
            cos_theta = np.cos(self.p_ang)
            sin_theta = np.sin(self.p_ang)
            dx = odom[0]*cos_theta - odom[1]*sin_theta
            dy = odom[0]*sin_theta + odom[1]*cos_theta
            self.p_xy += np.vstack((dx,dy)) + xy_noise

            # Increment angle
            self.p_ang += odom[2] + ang_noise
            self.p_ang = angle_wrap(self.p_ang)
            
            #Update flag for resampling
            self.moving=True     
    
    #===========================================================================
    def weight(self, lines):
        '''
        Look for the lines seen from the robot and compare them to the given map.
        Lines expressed as [x1 y1 x2 y2].
        '''
        # TODO: code here!!
        # Constant values for all weightings
        val_rng = 1.0 / (self.meas_rng_noise * np.sqrt(2 * np.pi))
        val_ang = 1.0 / (self.meas_ang_noise * np.sqrt(2 * np.pi))

        tunung_factor = 15

        # Loop over particles
        for i in range(self.num):
            map_polar = np.zeros((self.map.shape[0],2))

            # Transform map lines to local frame and to [range theta]
            for j in range(self.map.shape[0]):
                map_polar[j,:] = get_polar_line(self.map[j,:],[self.p_xy[0,i],self.p_xy[1,i],self.p_ang[i]])

            # Transform measured lines to [range theta] and weight them
            for j in range(lines.shape[0]):
                if ( (lines[j,0]-lines[j,2])**2 + (lines[j,1]-lines[j,3])**2 ) > 0:
                    local_map_polar = get_polar_line(lines[j,:])
                    w = 0
                    index = -1
                    # Weight them
                    for k in range(map_polar.shape[0]):
                        w_r = np.exp(-(map_polar[k,0]-local_map_polar[0])**2/(tunung_factor*2*self.odom_lin_sigma**2)) * val_rng
                        w_a = np.exp(-angle_wrap(map_polar[k,1]-local_map_polar[1])**2/(tunung_factor*2*self.odom_ang_sigma**2)) * val_ang
                        # w = np.maximum(w,w_r*w_a)
                        if w_r*w_a > w:
                            w = w_r * w_a
                            index = k

                    # self.p_wei[i] *= w
                 
                    # OPTIONAL question
                    # make sure segments correspond, if not put weight to zero
                    len_line_map = np.linalg.norm([(self.map[index,0]-self.map[index,2]), (self.map[index,1]-self.map[index,3])])
                    len_line_odom = np.linalg.norm([(lines[j,0]-lines[j,2]), (lines[j,1]-lines[j,3])])

                    # Check if measured line is larger than best fitting map line
                    # and set p_wei to 0 if it is (best match is invalid)
                    if len_line_odom > 1.1*len_line_map: # 10% error margin
                        # not 0 to avoid problems with turning all values to 0
                        self.p_wei[i] *= 0.00000000000000001
                    else:
                        self.p_wei[i] *= w
                

            
        # Normalize weights
        self.p_wei /= np.sum(self.p_wei)
        # TODO: Compute efficient number
        self.n_eff = 1.0/np.sum(self.p_wei**2)
        
    #===========================================================================
    def resample(self):
        '''
        Systematic resampling of the particles.
        '''
        # TODO: code here!!
        # Look for particles to replicate

        if self.moving == False: # or self.n_eff > self.num/4.0:
            return

        p_xy_new = np.zeros([2,self.num])
        p_ang_new = np.zeros(self.num)

        r = np.random.rand() / self.num
        c = self.p_wei[0]
        i = 0

        for m in range(self.num):
            u = r + (m * 1.0) / self.num
            while u > c:
                i += 1
                c += self.p_wei[i]

            p_xy_new[:,m] = self.p_xy[:,i]
            p_ang_new[m] = self.p_ang[i]
 
        self.p_xy = p_xy_new
        self.p_ang = p_ang_new
        self.p_wei = 1.0 / self.num * np.ones(self.num)
    
    #===========================================================================
    def get_mean_particle(self):
        '''
        Gets mean particle.
        '''
        # Weighted mean
        weig = np.vstack((self.p_wei, self.p_wei))
        mean = np.sum(self.p_xy * weig, axis=1) / np.sum(self.p_wei)
        
        ang = np.arctan2( np.sum(self.p_wei * np.sin(self.p_ang)) / np.sum(self.p_wei),
                          np.sum(self.p_wei * np.cos(self.p_ang)) / np.sum(self.p_wei) )
                          
        return np.array([mean[0], mean[1], ang])