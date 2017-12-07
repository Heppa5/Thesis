#!/usr/bin/env python 
import numpy as np
from matplotlib import pyplot as plt

import rospy
from geometry_msgs.msg import PoseStamped


import time
from pylab import *



import sys, select, os


def plot_x(msg):
    global counter
    global got_message
    global size
    global time_array
    global distance_array
    global start_time
    
    #print len(distance_array)
    for i in range(size-1,0,-1):
        local_shit=distance_array[i-1]
        distance_array[i]=local_shit
        local_shit2=time_array[i-1]
        time_array[i]=local_shit2
    # First element in list is the new one
    time_array[0]=time.time()-start_time
    distance_array[0]=msg.pose.position.x
    

    
    
    got_message=1


    counter += 1

if __name__ == '__main__':
    
    start_time=time.time()
    size = 100
    time_array=arange(0, size/10, 0.1);
    distance_array=arange(0, size/10, 0.1);
    
    counter = 0
    got_message=0 

    rospy.init_node("plotter")
    rospy.Subscriber("/test/euclidian_distance_markers", PoseStamped, plot_x)
    


    fig, ax = plt.subplots(1, 1)


    plt.show(False)
    xlabel("Time")
    ylabel("Euclidian distance (m) between markers")
    plt.draw()
    
    points = ax.plot(time_array, distance_array, 'r-')
    
    
    os.system('cls' if os.name == 'nt' else 'clear') 
    
    while(True) :
        if(got_message==1):
            got_message=0
            #print distance_array
            points[0].set_data(time_array, distance_array)
            plt.xlim([time_array[size-1],time_array[0]])
            plt.ylim([0,1.0])
            fig.canvas.draw()
          
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            sys.exit()

