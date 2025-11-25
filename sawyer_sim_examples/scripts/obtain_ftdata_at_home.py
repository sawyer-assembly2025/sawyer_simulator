#!/usr/bin/env python

"""
Created on Wed Nov 19 17:23:10 2025

@author: Hector Quijada

Example moves sawyer robot to predetermined home position and starts adquiring FT sensor data
End effector can be moved by hand

"""

import rospy
import rospkg
import time
import numpy as np
import os
from datetime import datetime
from intera_interface import robot_ctl_ik as robot
from intera_interface import ATI_Net as ati
from geometry_msgs.msg import WrenchStamped

#Global variables for force and torque
#Need to be global to be updated by call back function in subscriber
ati_ft_data = np.zeros((1,6))
ft_samples = np.zeros((1,6))

def obtain_ftdata(data):
	
    global ft_samples

    #Split wrench data
    ati_ft_data[0][0] = data.wrench.force.x
    ati_ft_data[0][1] = data.wrench.force.y
    ati_ft_data[0][2] = data.wrench.force.z
    ati_ft_data[0][3] = data.wrench.torque.x
    ati_ft_data[0][4] = data.wrench.torque.y
    ati_ft_data[0][5] = data.wrench.torque.z

    #Stack measurement at call back
    ft_samples = np.vstack((ft_samples,ati_ft_data))
    
def adquisition_cycle(test_time=1.0):

    global ft_samples

    try:
        rospy.Subscriber("/robot/ft_sensor_topic/", WrenchStamped, obtain_ftdata)

        #Define the duration you want the subscriber to run (e.g., 10 seconds)
        duration = rospy.Duration(test_time)
        start_time = rospy.Time.now()
        rospy.loginfo("Ft data adquisition started. It will run for %s seconds.", duration/1e9)

        # rospy.spin() would block until shutdown, so we use a loop
        while rospy.Time.now() - start_time < duration and not rospy.is_shutdown():
            # Sleep to prevent the loop from consuming all CPU resources
            rospy.Rate(100).sleep() # Adjust sleep time as needed

        final_time = rospy.Time.now() - start_time

        rospy.loginfo("Subscriber finished running for %s seconds. Saving data and shutting down.", final_time/1e9)
        
        #Delete first auxiliary value
        ft_samples = np.delete(ft_samples, 0, axis=0)

        #Save all data to a csv file
        current_datetime = datetime.now().strftime("%d%m%Y_%H_%M_%S")
        ATI_net_path = "samples_" + current_datetime + ".csv"
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('sawyer_sim_examples')
        csv_file_path = os.path.join(package_path, ATI_net_path)
        np.savetxt(csv_file_path, ft_samples, delimiter=',', fmt='%f')

        rospy.loginfo("Data saved succesfully")

        # Explicitly shutdown the node after the duration
        rospy.signal_shutdown("Finishing testing node")
    
    except rospy.ROSInterruptException:

        # Explicitly shutdown the node after the duration
        rospy.loginfo("Test unsuccesful, finishing testing node")


if __name__ == '__main__':

    try:
        #Initialize Example node
        rospy.init_node('sawyer_ftdata_home', anonymous=False)

        #Create sawyer robot instance
        sawyer = robot.SawyerRobot()

        #Wait 2 seconds
        time.sleep(2)

        #Move robot to home
        sawyer.move_to_home()

        #Wait 2 seconds
        time.sleep(2)

        #Start adquiring data
        adquisition_cycle(test_time=1.0)
    
    except rospy.ROSInterruptException:
        rospy.loginfo("Test unsuccesful, finishing testing node")