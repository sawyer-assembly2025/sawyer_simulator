#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from intera_interface import ATI_Net as ati

def publish_ftdata(sensor_ref):
    pub = rospy.Publisher('/robot/ati_ft_sensor_topic/', WrenchStamped, queue_size=1)
    rospy.init_node('ati_ftsensor_data_pub', anonymous=True)
    rate = rospy.Rate(100) # 100hz

    #Counter to confirm sampling
    i=0

    #Start ATI ft data broadcast
    sensor_ref.start_measuring()

    #Initialize auxiliary variable to publish data
    ft_wrench = WrenchStamped()

    #run while rospy is not shutdown
    while not rospy.is_shutdown():
        
        #Get sensor raw data
        raw_data = sensor_ref.get_data()
        
        #Put data in auxiliary data to publish
        ft_wrench.wrench.force.x = raw_data[0]
        ft_wrench.wrench.force.y = raw_data[1]
        ft_wrench.wrench.force.z = raw_data[2]
        ft_wrench.wrench.torque.x = raw_data[3]
        ft_wrench.wrench.torque.y = raw_data[4]
        ft_wrench.wrench.torque.z = raw_data[5]

        #Confirm connection
        if i == 0:
            rospy.loginfo("Connection established, ATI ft data sampling ongoing")

        pub.publish(ft_wrench)
        i += 1
        rate.sleep()

    #Stop ATI ft data broadcast
    sensor_ref.stop_measuring()

if __name__ == '__main__':
    try:
        #Initialize sensor
        delta = ati.DeltaFtSensor()

        #If no error while initializing start publishing data
        publish_ftdata(delta)

    except rospy.ROSInterruptException:
        pass
