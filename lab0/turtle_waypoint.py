#!/usr/bin/env python
import roslib; roslib.load_manifest('lab0_ros')
import rospy

#For command line arguments
import sys
#For atan2
import numpy as np

#TODO: Import the messages we need
##
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
##

#Initialization of turtle position
x=None
y=None
theta=None

#Position tolerance for both x and y
tolerance=0.1
#Have we received any pose msg yet?
gotPosition=False

def callback(pose_msg):
    global x,y,theta,gotPosition
    #TODO:Store the position in x,y and theta variables.
    x = pose_msg.x
    y = pose_msg.y
    theta = pose_msg.theta    
    gotPosition=True

def waypoint():
    global gotPosition
    #TODO: Define the pulisher: Name of the topic. Type of message
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)

    #Name of the node
    rospy.init_node('turtle_waypoint')

    #TODO: Define the subscriber: Name of the topic. Type of message. Callback function
    rospy.Subscriber("turtle1/pose", Pose, callback)

    #Has the turtle reach position?
    finished=False

    #If the point hasn't been specified in a command line:
    if(len(sys.argv)!=3):
        print('X and Y values not set or not passed correctly. Looking for default parameters.')
        #TODO: If ROS parameters default_x and default_y exist:
        if rospy.has_param('default_x') and rospy.has_param('default_y'): #Change this for the correct expression
            #TODO: Save them into this variables
            x_desired= rospy.get_param("default_x") #Change this for the correct expression
            y_desired= rospy.get_param("default_y") #Change this for the correct expression
            print('Heading to: %f,%f' %(x_desired, y_desired))
        else:
            print('Default values parameters not set!. Not moving at all')
            finished=true
    else:
        #Save the command line arguments.
        x_desired=float(sys.argv[1])
        y_desired=float(sys.argv[2])
        print('Heading to: %f,%f' %(x_desired, y_desired))

    # Control parameters for PD controller
    # note: control parameters could be pushed further if sleep time decreased or
    # if oscillatory behaviour is accepted
    # note2: integral channel not useful in this situation, would only lead to oscillations
    kp_v = 1.5
    kd_v = 0.2
    kp_w = 3
    kd_w = 1

    # Initialise variables to be used in the loop
    d_prev = 0
    dt_prev = 0

    vel_com = Twist()
    while not rospy.is_shutdown() and not finished:
        if(gotPosition):
            #TODO: Send a velocity command for every loop until the position is reached 
            # within the tolerance.

            # For proportional controller
            dx = x_desired - x # distance in x between turtle and target
            dy = y_desired - y # distance in y between turtle and target
            d = np.linalg.norm([dx,dy]) # total distance between turtle and target
            
            theta_target = np.arctan2(dy,dx) # angle to target
            dt = theta_target - theta # angle difference
            dt = np.mod(dt + np.pi,2 * np.pi) - np.pi # -pi <= dt < pi
            
            # For derivative controller
            d_deriv = d - d_prev
            d_prev = d
            dt_deriv = dt - dt_prev
            dt_prev = dt

            vel_com.linear.x = (kp_v*d + kd_v*d_deriv)*np.cos(dt/2)
            vel_com.angular.z = kp_w*dt + kd_w*dt
            pub.publish(vel_com)
            if d <= tolerance: #Change for the correct expression
                finished=True        

        #Publish velocity commands every 0.3 sec.
        rospy.sleep(0.3)

if __name__ == '__main__':
    try:
        waypoint()
    except rospy.ROSInterruptException:
        pass