#!/usr/bin/env python

# Author: Sanjuksha Nirgude

# This node recieves lidar data from /tim561_scan ROS topic 
# applies low pass filter on it
# publishes the filtered data to /tim561_scan_filtered 
import rospy
from sensor_msgs.msg import LaserScan 

# Initializing 
laserscan= None

RC = 0.2 # For low pass filter defined next
# Values of RC were changed to 0.2,5,10 to observe the variation
def lowpass(x, dt, RC): # Low pass filter 
    y =[0]*len(x)       # Initializing filtered data array
    a = dt/(RC+dt)      # alpha
    y[0] = a*x[0]
    for i in range(1,len(x)):
        y[i]=a*x[i]+(1-a)*y[i-1]  # filtering data
    return y

def callback(data): # callback function for each new laserscan value
    global laserscan
    laserscan= data

def main():
    
    rospy.init_node('test')  # to initiate and create an unique node 
    rospy.Subscriber('/tim561_scan', LaserScan, callback) # subscribing  to the topic from lidar_test_data.bag
    pub = rospy.Publisher('/tim561_scan_filtered', LaserScan, queue_size=10) # Publisher for filtered data 

    while laserscan is None and not rospy.is_shutdown(): # For when data is not available keeping the node on hold
        rospy.sleep(1)
    rate = rospy.Rate(10)  
    new_data=LaserScan() # Object for filtered data
    while not rospy.is_shutdown():
        new_data.header=laserscan.header  # Header is not changed in new data
        new_data.header.frame_id='laser' # ID changed for rviz visualization
        new_data.angle_min = laserscan.angle_min # Doesn't change after filtering 
        new_data.angle_max = laserscan.angle_max  # Doesn't change after filtering
        new_data.angle_increment = laserscan.angle_increment  # Doesn't change after filtering
        new_data.time_increment = laserscan.time_increment  # Doesn't change after filtering
        
        new_data.ranges=lowpass(laserscan.ranges,laserscan.angle_increment,RC) # Filtering the data
        new_data.range_min = min(new_data.ranges) # Finding new minimum of filtered data
        new_data.range_max = max(new_data.ranges) # Finding new maximum of filtered data
        
        new_data.intensities=laserscan.intensities # Doesn't change after filtering(empty)
        pub.publish(new_data) # Publishing filtered data
        rate.sleep()          
if __name__ == "__main__":
    main()