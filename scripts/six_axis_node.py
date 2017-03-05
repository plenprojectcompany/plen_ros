#!/usr/bin/env python
# coding=utf-8

__author__    = 'Mitsuhiro YABU'
__author__    = 'Tatsuroh SAKAGUCHI'
__author__    = 'Yugo KAJIWARA'
__copyright__ = 'PLEN Project Company, and all authors.'
__license__   = 'BSD'

import struct

import mraa
import rospy
from geometry_msgs.msg import Accel
from std_msgs.msg import Empty


class Mpu(object):
    I2C_PORT = 0
    I2C_ADDRESS = 0x68

    def __init__(self):
        self._i2c = mraa.I2c(Mpu.I2C_PORT)
        self._i2c.address(Mpu.I2C_ADDRESS)

        # Read WHO_AM_I register (0x75) of device at address.
        self._i2c.writeByte(0x75)
        self._i2c.readByte()

        # Set power management register (0x6B) to use gyro clock (0x01). 
        self._i2c.writeReg(0x6B, 0x01)
        # Set configuration register (0x1A) filter at 90Hz (0x02).
        self._i2c.writeReg(0x1A, 0x02)
        # Set gyro configuration register (0x1B) to all zeros (0x00).
        self._i2c.writeReg(0x1B, 0x00)
        # Set accelerometer configuration register (0x1C) to all zeros (0x00).
        self._i2c.writeReg(0x1C, 0x00)

    def read_accelgyros(self):
        # Get acceleration Data
        acc_start_add = 0x3B
        acc_nbytes = 6
        acc_reg_addresses = xrange(acc_start_add, acc_start_add + acc_nbytes)
        i2c_data = map(self._i2c.readReg, acc_reg_addresses)

        # Get gyro Data
        gyr_start_add = 0x43
        gyr_nbytes = 6
        gyr_reg_addresses = xrange(gyr_start_add, gyr_start_add + gyr_nbytes)
        i2c_data.extend(map(self._i2c.readReg, gyr_reg_addresses))

        # '>' :  big endian
        # 'h' :  2byte signed integer
        data_format = '>' + 'h' * ((acc_nbytes + gyr_nbytes) / struct.calcsize('h'))
        assert struct.calcsize(data_format) == (acc_nbytes + gyr_nbytes)
        accelgyros = struct.unpack(data_format, bytes(bytearray(i2c_data)))

        return accelgyros


class Node(object):
    NAME = 'six_axis_node'
    PUBLISHER_NAME = 'accel'
    SUBSCRIBER_NAME = 'request_accel'
    ROSPY_RATE_HZ = 10

    def __init__(self):
        self.mpu = Mpu()

        rospy.init_node(Node.NAME, anonymous=True)
        self.publisher = rospy.Publisher(
            Node.PUBLISHER_NAME, Accel, queue_size=10)
        rospy.Subscriber(Node.SUBSCRIBER_NAME, Empty, self.subscribe)
        self.rospy_rate = rospy.Rate(Node.ROSPY_RATE_HZ)
        self.auto_publish = True

    def subscribe(self, message):
        rospy.loginfo('GET REQUEST')
        self.auto_publish = False

        self.publish()

    def publish(self):
        rospy.loginfo('PUBLISH ACCEL')
        accelgyro = self.mpu.read_accelgyros()

        # Set acceleration data and gyro data (Normalize for PLEN Axis)
        response = Accel()
        response.linear.x = accelgyro[0]
        response.linear.y = -accelgyro[1]
        response.linear.z = -accelgyro[2]
        response.angular.x = accelgyro[3]
        response.angular.y = -accelgyro[4]
        response.angular.z = -accelgyro[5]
        self.publisher.publish(response)

    def start(self):
        try:
            while not rospy.is_shutdown():
                if self.auto_publish:
                    self.publish()
                self.rospy_rate.sleep()
        finally:
            pass


if __name__ == '__main__':
    Node().start()
