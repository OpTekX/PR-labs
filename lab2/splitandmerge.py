#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
from probabilistic_lib.functions import angle_wrap

#===============================================================================
def splitandmerge(points, split_thres=0.1, inter_thres=0.3, min_points=6, dist_thres=0.12, ang_thres=np.deg2rad(10)):
# def splitandmerge(points, split_thres=0.01, inter_thres=0.3, min_points=6, dist_thres=0.12, ang_thres=np.deg2rad(10)):
    '''
    Takes an array of N points in shape (2, N) being the first row the x
    position and the second row the y position.

    Returns an array of lines of shape (L, 4) being L the number of lines, and
    the columns the initial and final point of each line in the form
    [x1 y1 x2 y2].

    split_thres: distance threshold to provoke a split
    inter_thres: maximum distance between consecutive points in a line
    min_point  : minimum number of points in a line
    dist_thres : maximum distance to merge lines
    ang_thres  : maximum angle to merge lines
    '''
    if points.shape[1] == 0:
        return np.array([[0, 0, 0, 0]])
    lines = split(points, split_thres, inter_thres, min_points, 0, points.shape[1]-1)
    return merge(lines, dist_thres, ang_thres)
    
#===============================================================================
def split(points, split_thres, inter_thres, min_points, first_pt, last_pt):
    '''
    Find lines in the points provided.
    first_pt: column position of the first point in the array
    last_pt : column position of the last point in the array
    '''

    # TODO: CODE HERE!!!
    # Check minimum number of points
    if (last_pt-first_pt+1) < min_points:
        return None

    # dist_vec = np.vectorize(dist)
    # global dist_vec
    # Line defined as "a*x + b*y + c = 0"
    # Calc (a, b, c) of the line (prelab question)
    x1 = points[0, first_pt]
    y1 = points[1, first_pt]
    x2 = points[0, last_pt]
    y2 = points[1, last_pt]
    a = y1 - y2
    b = x2 - x1
    c = x1*y2 - x2*y1
    

    # Distances of points to line (prelab question)
    x0 = points[0,first_pt:last_pt]
    y0 = points[1,first_pt:last_pt]
    distances = np.abs(a*x0 + b*y0  + c)/np.sqrt(a*a+b*b)
    max_dist = np.max(distances)
    

    # Check split threshold
    if max_dist > split_thres:
       
        # Check sublines
        P_ind = first_pt + np.where(distances==max_dist)[0][0]
        prev = split(points, split_thres, inter_thres, min_points, first_pt, P_ind)
        post = split(points, split_thres, inter_thres, min_points, P_ind+1, last_pt)
       
        # Return results of sublines
        if prev is not None and post is not None:
            return np.vstack((prev, post))
        elif prev is not None:
            return prev
        elif post is not None:
            return post
        else:
            return None

    # # Do not need to split furthermore
    else:
        # Optional check interpoint distance
        for i in range(first_pt, last_pt):
            xa = points[0, i]
            ya = points[1, i]
            xb = points[0, i+1]
            yb = points[1, i+1]
            d = np.linalg.norm([(xa-xb), (ya-yb)])
            # Check interpoint distance threshold
            if d > inter_thres:
                #Split line
                prev = split(points, split_thres, inter_thres, min_points, first_pt, i)
                post = split(points, split_thres, inter_thres, min_points, i+1, last_pt)
               
                # Return results of sublines
                if prev is not None and post is not None:
                    return np.vstack((prev, post))
                elif prev is not None:
                    return prev
                elif post is not None:
                    return post
                else:
                    return None

        # It is a good line
        return np.array([[x1, y1, x2, y2]])
        
    # Dummy answer --> delete it
    # return np.array([[x1, y1, x2, y2]])

#===============================================================================
def merge(lines, dist_thres, ang_thres):
    '''
    Merge similar lines according to the given thresholds.
    '''
    # No data received
    if lines is None: # or lines.shape[1] < 4:
        return np.array([])

    # if lines.shape[0]<2:
    #     return lines

        
    # Check and merge similar consecutive lines
    i = 0
    while i < lines.shape[0]-1:
        
        # Line angles
        ang1 = np.arctan2(lines[i][0]-lines[i][2],lines[i][1]-lines[i][3])
        ang2 = np.arctan2(lines[i+1][0]-lines[i+1][2],lines[i+1][1]-lines[i+1][3])
        
        # Below thresholds?
        angdiff = np.abs(angle_wrap(ang1 - ang2))
        disdiff = np.linalg.norm([(lines[i][2]-lines[i+1][0]), (lines[i][3]-lines[i+1][1])])
        if angdiff < ang_thres and disdiff < dist_thres:
            
            # Joined line
            lines[i,:] = np.array([[lines[i][0], lines[i][1], lines[i+1][2], lines[i+1][3]]])
            
            # Delete unnecessary line
            lines = np.delete(lines, i+1, 0)

        # Nothing to merge
        else:
            i += 1
            
    return lines

# def dist(x0,y0,a,b,c): 
#     return np.abs(a*x0 + b*y0  + c)/np.sqrt(a*a+b*b)

# dist_vec = np.vectorize(dist)
