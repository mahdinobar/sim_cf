#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory
import math
from geometry_msgs.msg import Twist
from crazyflie_driver.msg import Position
from crazyflie_driver.msg import GenericLogData

def local_position_callback(msg):
    global current_position 
    global next_pos
    current_position.x = msg.values[0]
    current_position.y = msg.values[1]
    current_position.z = msg.values[2]
    current_position.header = msg.header


if __name__ == '__main__':
    rospy.init_node('test_command')

    cf_1 = crazyflie.Crazyflie("cf1", "/cf1")
    # cf_2 = crazyflie.Crazyflie("cf2", "/cf2")
    # cf_3 = crazyflie.Crazyflie("cf3", "/cf3")
    # cf_4 = crazyflie.Crazyflie("cf4", "/cf4")

    #take-off
    cf_1.setParam("commander/enHighLevel", 1)
    # cf_2.setParam("commander/enHighLevel", 1)
    # cf_3.setParam("commander/enHighLevel", 1)
    # cf_4.setParam("commander/enHighLevel", 1)

    cf_1.takeoff(targetHeight = 1.5, duration = 2.0)
    # cf_2.takeoff(targetHeight = 1.5, duration = 2.0)
    # cf_3.takeoff(targetHeight = 1.5, duration = 2.0)
    # cf_4.takeoff(targetHeight = 1.5, duration = 2.0)
    time.sleep(5.0)

    global freq_pub
    freq_pub = 100
    rate = rospy.Rate(freq_pub)
    #velocity command
    #pubSetpointVel = rospy.Publisher("/cf1" +"/cmd_vel", Twist , queue_size=10)
    rospy.Subscriber("/cf1" + "/local_position" , GenericLogData , local_position_callback)
    pubSetpointPos = rospy.Publisher("/cf1" +"/cmd_position", Position , queue_size=10)
    next_pos = Position()
    current_position = Position()



    #vel_msg = Twist()
    #vel_msg.linear.x=1.
    #vel_msg.linear.y=0.
    #vel_msg.linear.z=0.
    #vel_msg.angular.x = 0.
    #vel_msg.angular.y = 0.
    #vel_msg.angular.z = 0.
    

    a=0
    while a<1e5:
        next_pos.x = 0.5/freq_pub + current_position.x
        next_pos.y = 0./freq_pub + current_position.y
        next_pos.z = 0./freq_pub + current_position.z
        next_pos.header.seq += 1
        next_pos.header.stamp = rospy.Time.now()
        #pubSetpointVel.publish(vel_msg)
        pubSetpointPos.publish(next_pos)
        a+=1
        # print('a',a)
        # print('current_position=',current_position)
        # print('next_pos=',next_pos)
        rate.sleep()
    
    time.sleep(3.0)


    #land
    cf_1.land(targetHeight = 0.0, duration = 2.0)
    # cf_2.land(targetHeight = 0.0, duration = 2.0)
    # cf_3.land(targetHeight = 0.0, duration = 2.0)
    # cf_4.land(targetHeight = 0.0, duration = 2.0)
    time.sleep(3.0)
    

    cf_1.stop()
    # cf_2.stop()
    # cf_3.stop()
    # cf_4.stop()
