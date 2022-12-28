#!/usr/bin/env python
#-*- coding: utf-8 -*-
from imu_driver.msg import imu_msg
import serial
import rospy
import utm
from std_msgs.msg import String
import numpy as np 
import sys
import time
rospy.init_node('mynode')
port = rospy.get_param("~port_number")

ser = serial.Serial(port=port, baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
pub = rospy.Publisher('imu', imu_msg, queue_size=10)

t = rospy.Time.now()
#t2= t.split(".")
ser.write("VNWRG,06,0*XX".encode('utf-8'))
ser.write("VNWRG,07,40*XX".encode('utf-8'))

rate = rospy.Rate(200) # 10hz
msg = imu_msg()


while True: 
    sac = str(ser.readline())
    
    if 'YMR' in str(sac):
        pl = sac.split(",")
        print(sac)
        yaw = float(pl[1])*(np.pi/180)
        pitch = float(pl[2])*(np.pi/180)
        roll = float(pl[3])*(np.pi/180)
        mag_x = float(pl[4])
        mag_y = float(pl[5])
        mag_z = float(pl[6])
        accl_x = float(pl[7])
        accl_y = float(pl[8])
        accl_z = float(pl[9])
        angl_x = float(pl[10])
        angl_y = float(pl[11])
        angl_z1 =pl[12]
        angl_z = float(angl_z1[:-8])
        
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        msg.Header.stamp.secs = t.secs
        msg.Header.stamp.nsecs = t.nsecs
        #print(t.to_nsec(), t.to_sec())
        msg.Header.frame_id = "IMU1_Frame"
        msg.IMU.orientation.x = qx
        msg.IMU.orientation.y = qy
        msg.IMU.orientation.z = qz
        msg.IMU.orientation.w = qw
        msg.IMU.angular_velocity.x = angl_x
        msg.IMU.angular_velocity.y = angl_y
        msg.IMU.angular_velocity.z = angl_z
        msg.IMU.linear_acceleration.x= accl_x
        msg.IMU.linear_acceleration.y= accl_y
        msg.IMU.linear_acceleration.z= accl_z
        msg.MagField.magnetic_field.x = mag_x
        msg.MagField.magnetic_field.y = mag_y
        msg.MagField.magnetic_field.z = mag_z
        msg.VNYMR = sac
    
        pub.publish(msg)




