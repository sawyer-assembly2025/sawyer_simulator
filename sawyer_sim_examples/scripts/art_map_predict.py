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
from intera_interface import ART
from intera_interface import ATI_Net as ati
from geometry_msgs.msg import WrenchStamped

#Global variables for force and torque
#Need to be global to be updated by call back function in subscriber
ati_ft_data = np.zeros((1,6))

def obtain_ftdata(data):

    #Split wrench data
    ati_ft_data[0][0] = data.wrench.force.x
    ati_ft_data[0][1] = data.wrench.force.y
    ati_ft_data[0][2] = data.wrench.force.z
    ati_ft_data[0][3] = data.wrench.torque.x
    ati_ft_data[0][4] = data.wrench.torque.y
    ati_ft_data[0][5] = data.wrench.torque.z
    
def adquisition_cycle(artmap, robot):
    
    global ati_ft_data

    i = 0
    rot_step = 0.001
    tran_step = 0.0001

    pred = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [tran_step, 0.0, 0.0, 0.0, 0.0, 0.0],
            [-tran_step, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, tran_step, 0.0, 0.0, 0.0, 0.0],
            [0.0, -tran_step, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, tran_step, 0.0, 0.0, 0.0],
            [0.0, 0.0, -tran_step, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, rot_step, 0.0, 0.0],
            [0.0, 0.0, 0.0, -rot_step, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, rot_step, 0.0],
            [0.0, 0.0, 0.0, 0.0, -rot_step, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, rot_step],
            [0.0, 0.0, 0.0, 0.0, 0.0, -rot_step],
            [tran_step, tran_step, 0.0, 0.0, 0.0, 0.0],
            [tran_step, -tran_step, 0.0, 0.0, 0.0],
            [-tran_step, tran_step, 0.0, 0.0, 0.0],
            [-tran_step, -tran_step, 0.0, 0.0, 0.0, 0.0]
            ]

    try:
        rospy.Subscriber("/robot/ft_sensor_topic/", WrenchStamped, obtain_ftdata)

        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            
            j = artmap.predict(ati_ft_data)

            robot.move_to_cartesian_relative(position=[pred[j][2],pred[j][1],pred[j][2]],orientation=[pred[j][3],pred[j][4],pred[j][5]],linear_speed=0.01)

            i += 1

        final_time = rospy.Time.now() - start_time
        rospy.loginfo("Total predictions: %s", i)
        rospy.loginfo("Finished testing, shutting down.", final_time)
    
    except rospy.ROSInterruptException:

        rospy.signal_shutdown("Finishing testing node")


if __name__ == '__main__':
    
    try:
        #Initialize Example node
        rospy.init_node('sawyer_ftdata_home', anonymous=False)

        #Create sawyer robot instance
        sawyer = robot.SawyerRobot()

        artmap = ART.FuzzyArtMap()

        #Train Fuzzy ARTMAP (Give dimensions before adding complement)
        wa_dim, wb_dim, wab_dim = artmap.train(Ia_dim=6, Ib_dim=4, train_path="train.csv", save_weights=True)

        #Load trained weights
        artmap.load_weights(wa_dim, wb_dim, wab_dim)
    
    
        for i in range(I.shape[0]):
            start_time = time.perf_counter()
            category_predicted = artmap.predict(I[i])
            print(f"Input {i+1} is clustered at neuron: {category_predicted}")
            end_time = time.perf_counter()
            execution_time = end_time - start_time
            print(f"Execution time: {execution_time:.6f} seconds")
            

        #Wait 2 seconds
        time.sleep(2)

        if sawyer._is_clicksmart == True:
            sawyer.set_red_light()
        
        #Move robot to home
        sawyer.move_to_home()

        #Wait 2 seconds
        time.sleep(2)

            
        if sawyer._is_clicksmart == True:
                sawyer.set_green_light()
            
        #Start adquiring data
        adquisition_cycle(artmap=artmap, robot=sawyer)
            
        if sawyer._is_clicksmart == True:
            sawyer.set_blue_light()
            
        #Wait 2 seconds
        time.sleep(2)
            
        # Explicitly shutdown the node after the duration
        rospy.signal_shutdown("Finishing testing node")
    
    except rospy.ROSInterruptException:
        
        # Explicitly shutdown the node after the duration
        rospy.signal_shutdown("Finishing testing node")