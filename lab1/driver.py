#!/usr/bin/env python

import os

# ROS general imports
import roslib
roslib.load_manifest('lab1_turtlebot')
import rospy

# Other imoprts
import math #math library
import numpy as np #numpy library
from probabilistic_lib.functions import angle_wrap #Normalize angles between -pi and pi

#TODO import the library to read csv
import csv

#TODO import the library to compute transformations
from tf.transformations import euler_from_quaternion

#ROS messages
#TODO import appropiate ROS messages
from geometry_msgs.msg import Twist # For
from nav_msgs.msg import Odometry

class driver(object):
    
    def __init__(self):
        '''
        constructor
        '''
        # print(os.getcwd())
        #Initialize ros node
        rospy.init_node('turtlebot_driver')        
        
        #Initialize goals
        self.x = np.array([])
        self.y = np.array([])
        self.theta = np.array([])
        
        #Threshold for distance to goal
        self.goal_th_xy = rospy.get_param('goal_thershold_xy',0.1) #Position threshold
        self.goal_th_ang = rospy.get_param('goal_threshold_ang',0.01) #Orientation threshold
        
        #Point to the first goal
        self.active_goal = 0

        #Initialize number of goals
        self.num_goals = 1
        
        #TODO define subscriber
        self.sub = rospy.Subscriber("/odom", Odometry, self.callback)
        
        #TODO define publisher        
        self.pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=100)
        
        #TODO define the velocity message
        self.vmsg = Twist()

        #Controller parameters
        self.kp_v = 0.2
        self.kp_w = 0.2
        self.kd_v = 0.5
        self.kd_w = 0.5

        #Controller variables
        self.d_prev = 0
        self.dt_prev = 0

        #Has the goal been loaded?
        self.params_loaded = False            

    def print_goals(self):
        '''
        print_goals prints the list of goals
        '''
        rospy.loginfo("List of goals:")
        rospy.loginfo("X:\t " + str(self.x))
        rospy.loginfo("Y:\t " + str(self.y))
        rospy.loginfo("Theta:\t " + str(self.theta))
    
    def print_goal(self):
        '''
        pritn_goal prints the next goal
        '''
        rospy.loginfo( "Goal: (" + str(self.x[self.active_goal]) + " , " + str(self.y[self.active_goal]) + " , " + str(self.theta[self.active_goal]) + ")")
        
    def print_pose(self):
        '''
        print_pose pints the robot's actual position
        '''
        rospy.loginfo( "Pose: (" + str(self.position_x) + " , " + str(self.position_y) + " , " + str(self.position_theta) + " )")
        
    def print_goal_and_pose(self):
        '''
        pritn_goal_and_pose prints both goal and pose
        '''
        rospy.loginfo("\tPose\t\tGoal")
        rospy.loginfo("X:\t%f\t%f",self.position_x,self.x[self.active_goal])
        rospy.loginfo("Y:\t%f\t%f",self.position_y,self.y[self.active_goal])
        rospy.loginfo("A:\t%f\t%f",self.position_theta,self.theta[self.active_goal])
        
    def dist_to_goal_xy(self):
        '''
        dist_to_goal_xy computes the distance in x and y direction to the 
        active goal
        '''
        return math.sqrt(pow(self.position_x-self.x[self.active_goal],2)+pow(self.position_y-self.y[self.active_goal],2))
    
    def dist_to_goal_ang(self):
        '''
        dist_to_goal_ang computes the orientation distance to the active
        goal
        '''
        return np.abs(angle_wrap(self.theta[self.active_goal]-self.position_theta))
        
    def has_arrived_xy(self):
        '''
        has_arrived_xy returns true if the xy distance to the ative goal is
        smaller than the position threshold
        '''
        return self.dist_to_goal_xy()<self.goal_th_xy
        
    def has_arrived_ang(self):
        '''
        has_arrived_ang returns true if the orientation distance to the 
        ative goal is smaller than the orientation threshold
        '''
        return self.dist_to_goal_ang()<self.goal_th_ang
        
    def has_arrived(self):
        '''
        has_arrived return true if the robot is closer than the apropiate 
        threshold to the goal in position and orientation
        '''
        return (self.has_arrived_xy() and self.has_arrived_ang())   
    
    def check_goal(self):
        '''
        check_goal checks if the robot has arrived to the active goal, 
        '''
        if self.has_arrived():
            self.next_goal()
        
    def publish(self):
        '''
        publish publish the velocity message in vmsg
        '''
        self.pub.publish(self.vmsg)
        
    def callback(self,msg):
        '''
        callback reads the actuall position of the robot, computes the 
        appropiate velocity, publishes it and check if the goal is reached
        '''
        self.read_position(msg)
        self.compute_velocity()
        self.publish()
        self.check_goal()
        
    def drive(self):
        '''
        drive is a neede function for the ros to run untill somebody stops
        the driver
        '''
        self.print_goal()
        while not rospy.is_shutdown():
            rospy.sleep(0.03)
        
    def load_goals(self):
        '''
        load_goals loads the goal (or goal list for the option al part) into
        the x y theta variables.
        
        TODO modify for the optional part
        '''
        filepath = rospy.get_param('file',0)
        if filepath:
            loaded = False
            try:
                f = open(filepath)
                print("Loaded coordinates file")
                loaded = True
            except IOError as e:
                print("IO Error ({0}): {1}".format(e.errno, e.strerror))
                print("Coordinates file not found")

            if loaded:
                line = f.readline().strip()
                while line:
                    line_vals = line.rsplit(',')
                    self.x = np.append(self.x,float(line_vals[0]))
                    self.y = np.append(self.y,float(line_vals[1]))
                    self.theta = np.append(self.theta,float(line_vals[2]))
                    line = f.readline().strip()
            else:
                self.x = np.append( self.x,rospy.get_param('x',0))
                self.y = np.append( self.y,rospy.get_param('y',0))
                self.theta = np.append( self.theta,rospy.get_param('theta',0))

        else:
            print('no file parameter')
            self.x = np.append( self.x,rospy.get_param('x',0))
            self.y = np.append( self.y,rospy.get_param('y',0))
            self.theta = np.append( self.theta,rospy.get_param('theta',0))

        self.num_goals = self.x.size
        self.params_loaded = True
        self.print_goals()
        
    def next_goal(self):
        '''
        next_goal increments the index of the goal in 1 and checks whether
        or not the robot has reached the final goal
        
        TODO modify for the optional part
        '''
        print('Loading next goal')
        self.active_goal = self.active_goal + 1
        if self.active_goal >= self.num_goals:
            rospy.signal_shutdown('Final goal reached!')
            self.active_goal = self.active_goal - 1 # Just in case
        self.print_goal()
        
    def read_position(self,msg):
        '''
        read_position copy the position received in the message (msg) to the
        internal class variables self.position_x, self.position_y and 
        self.position_theta
        
        Tip: in order to convert from quaternion to euler angles give a look
        at tf.transformations
        
        TODO implement!
        '''
        ori = msg.pose.pose.orientation # Extract orientation object
        quat = [ori.x, ori.y, ori.z, ori.w] # Transform into appropriate form
        angles = euler_from_quaternion(quat) # Get angles from quaternion

        # Save position into object variables
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.position_theta = angles[2]
        
    def compute_velocity(self):
        '''
        compute_velocity computes the velocity which will be published.
        
        TODO implement!
        '''
        dx = self.x[self.active_goal] - self.position_x # distance in x between turtle and target
        dy = self.y[self.active_goal] - self.position_y # distance in y between turtle and target
        d = np.linalg.norm([dx,dy]) # total distance between turtle and target

        if d > self.goal_th_xy/2: # Using threshold/2 to avoid entering and leaving goal radius
            theta_target = np.arctan2(dy,dx) # angle to target
            dt = angle_wrap(theta_target - self.position_theta) # angle difference

            # For derivative controller
            d_deriv = d - self.d_prev
            self.d_prev = d
            dt_deriv = dt - self.dt_prev
            self.dt_prev = dt

            # Calculate control signals
            linv = (self.kp_v*d + self.kd_v*d_deriv)*np.cos(dt/2)
            self.vmsg.linear.x = np.min([linv/np.sqrt(np.abs(linv)),0.2])
            angv = self.kp_w*dt + self.kd_w*dt_deriv
            self.vmsg.angular.z = angv/np.sqrt(np.abs(angv))

        elif not self.has_arrived_ang():
            dt = angle_wrap(self.theta[self.active_goal] - self.position_theta) # angle difference

            # For derivative controller
            dt_deriv = dt - self.dt_prev
            self.dt_prev = dt

            # Calculate control signals
            self.vmsg.linear.x = 0
            angv = self.kp_w*dt + self.kd_w*dt_deriv
            self.vmsg.angular.z = angv/np.sqrt(np.abs(angv))
