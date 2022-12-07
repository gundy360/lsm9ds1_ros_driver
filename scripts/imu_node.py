#!/usr/bin/env python

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Simple demo of the LSM9DS1 accelerometer, magnetometer, gyroscope.
# Will print the acceleration, magnetometer, and gyroscope values every second.






# Main loop will read the acceleration, magnetometer, gyroscope, Temperature
# values every second and print them out.
#while True:
    # Read acceleration, magnetometer, gyroscope, temperature.
    
    
    # Print values.
#    print(
#        "Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})".format(
#            accel_x, accel_y, accel_z
#        )
#    )
#    print(
#        "Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})".format(mag_x, mag_y, mag_z)
#    )
#    print(
#        "Gyroscope (rad/sec): ({0:0.3f},{1:0.3f},{2:0.3f})".format(
#            gyro_x, gyro_y, gyro_z
#        )
#    )
#    print("Temperature: {0:0.3f}C".format(temp))
    # Delay for a second.
#    time.sleep(1.0)

import time
#import smbus
import board
import adafruit_lsm9ds1
import struct
import rospy
import numpy as np
from sensor_msgs.msg import Temperature, Imu, MagneticField
from tf.transformations import quaternion_about_axis
import Wireframe_EKF as wf
#from lsm9ds1_driver.registers import PWR_MGMT_1, ACCEL_XOUT_H, ACCEL_YOUT_H, ACCEL_ZOUT_H, TEMP_H,\
#    GYRO_XOUT_H, GYRO_YOUT_H, GYRO_ZOUT_H

ADDR = None
bus = None
IMU_FRAME = None
IMU_Update_Time_Sec = 0.02

# read_word and read_word_2c from http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html
#def read_word(adr):
#    high = bus.read_byte_data(ADDR, adr)
#    low = bus.read_byte_data(ADDR, adr+1)
#    val = (high << 8) + low
#    return val

#def read_word_2c(adr):
#    val = read_word(adr)
#    if (val >= 0x8000):
#        return -((65535 - val) + 1)
#    else:
#        return val

def publish_temp(timer_event):
    temp_msg = Temperature()
    temp_msg.header.frame_id = IMU_FRAME
    temp_msg.temperature = sensor.temperature
    temp_msg.header.stamp = rospy.Time.now()
    temp_pub.publish(temp_msg)

def publish_mag(timer_event):
    mag_x, mag_y, mag_z = sensor.magnetic

    mag_msg = MagneticField()
    mag_msg.header.frame_id = IMU_FRAME
    mag_msg.header.stamp = rospy.Time.now()
    mag_msg.magnetic_field.x = mag_x
    mag_msg.magnetic_field.y = mag_y
    mag_msg.magnetic_field.z = mag_z
    #mag_msg.magnetic_field_covariance = [0 0 0 0 0 0 0 0 0]
    mag_pub.publish(mag_msg)

def publish_imu(timer_event):
    imu_msg = Imu()
    imu_msg.header.frame_id = IMU_FRAME
    
    accel_x, accel_y, accel_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
    gyro_x, gyro_y, gyro_z = sensor.gyro

    Imu_Ekf.quatRotate([gyro_x, gyro_y, gyro_z],
                 [accel_x, accel_y, accel_z],
                 [mag_x, mag_y, mag_z],
                 1/IMU_Update_Time_Sec)
    #q1, q2, q3, q4 = Imu_Ekf.getAttitudeQuaternion()
#  sendToPC(&gyroData.X, &gyroData.Y, &gyroData.Z, 
#           &accelData.X, &accelData.Y, &accelData.Z,
#           &magData.X, &magData.Y, &magData.Z);

    # Read the acceleration vals
    #accel_x = read_word_2c(ACCEL_XOUT_H) / 16384.0
    #accel_y = read_word_2c(ACCEL_YOUT_H) / 16384.0
    #accel_z = read_word_2c(ACCEL_ZOUT_H) / 16384.0
    
    # Calculate a quaternion representing the orientation
    accel = accel_x, accel_y, accel_z
    ref = np.array([0, 0, 1])
    acceln = accel / np.linalg.norm(accel)
    axis = np.cross(acceln, ref)
    angle = np.arccos(np.dot(acceln, ref))
    orientation = quaternion_about_axis(angle, axis)

    # Read the gyro vals
    #gyro_x = read_word_2c(GYRO_XOUT_H) / 131.0
    #gyro_y = read_word_2c(GYRO_YOUT_H) / 131.0
    #gyro_z = read_word_2c(GYRO_ZOUT_H) / 131.0
    
    # Load up the IMU message
    o = imu_msg.orientation
    #o.x, o.y, o.z, o.w = orientation
    o.x, o.y, o.z, o.w = Imu_Ekf.getAttitudeQuaternion()

    imu_msg.linear_acceleration.x = accel_x
    imu_msg.linear_acceleration.y = accel_y
    imu_msg.linear_acceleration.z = accel_z

    imu_msg.angular_velocity.x = gyro_x
    imu_msg.angular_velocity.y = gyro_y
    imu_msg.angular_velocity.z = gyro_z

    imu_msg.header.stamp = rospy.Time.now()

    imu_pub.publish(imu_msg)


temp_pub = None
imu_pub = None
mag_pub = None

if __name__ == '__main__':
    rospy.init_node('imu_node')

    # Create sensor object, communicating over the board's default I2C bus
    i2c = board.I2C()  # uses board.SCL and board.SDA
    sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c,0x1C,0x6A) #Override addresses for Sense Hat
    
    # Create sensor fusion object
    Imu_Ekf = wf.Wireframe()
	
    #bus = smbus.SMBus(rospy.get_param('~bus', 1))
    #ADDR = rospy.get_param('~device_address', 0x68)
    #if type(ADDR) == str:
    #    ADDR = int(ADDR, 16)

    IMU_FRAME = rospy.get_param('~imu_frame', 'imu_link')

    #bus.write_byte_data(ADDR, PWR_MGMT_1, 0)

    temp_pub = rospy.Publisher('temperature', Temperature)
    imu_pub = rospy.Publisher('imu/data', Imu)
    mag_pub = rospy.Publisher('imu/mag', MagneticField)
    imu_timer = rospy.Timer(rospy.Duration(IMU_Update_Time_Sec), publish_imu)
    temp_timer = rospy.Timer(rospy.Duration(10), publish_temp)
    mag_timer = rospy.Timer(rospy.Duration(IMU_Update_Time_Sec), publish_mag)
    rospy.spin()
