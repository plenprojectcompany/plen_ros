#!/usr/bin/env python
import struct

import serial
import mraa
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometory_msgs.msg import Accel

class PlenSerial(serial.Serial):

    def __init__(self, port='/dev/ttyMFD1', baudrate=115200, re_de_pin=36, **kwargs):
        super(PlenSerial, self).__init__(
            port=port, baudrate=baudrate, **kwargs)
        self.re_de = mraa.Gpio(re_de_pin)

    def write_with_re_de(self, text):
        self.re_de.write(1)
        try:
            self.write(text)
            self.flushOutput()
        finally:
            self.re_de.write(0)


class Node(object):
    SLEEP_RATE_HZ = 50

    def __init__(self):
        self.serial = PlenSerial()

        rospy.init_node('serial_node', anonymous=True)
        self.subscribers = (
            rospy.Subscriber('to_rs485', String, self.subscribe_rs485),
            rospy.Subscriber('accel', Accel, self.subscribe_accel),
        )
        self.publishers = (
            rospy.Publisher('request_accel', Empty, queue_size = 10),
            rospy.Publisher('from_rs485', String, queue_size = 10),
        )
        self.sleep_rate = rospy.Rate(self.SLEEP_RATE_HZ)

    def subscribe_rs485(self, message):
        _, data = message.data.split(',')
        #header = '>DV'+str(device_id, 16)+str(len(data), 16)
        #self.serial.write_with_re_de(header+data)
        self.serial.write_with_re_de(data)

    def subscribe_accel(self, message):
        self.accel = message

    def request_accel(self, message):
        rospy.loginfo('REQUEST ACCEL')
        request = Empty()
        self.publishers[0].publish(request)

    def write_accel(self):
        # ready?
        if self.serial.inWaiting() <= 0:
            return
        data = self.serial.read()
        if data != '>':
            rospy.logwarn('invalid input')

        accelgyros = str(self.accel.linear.x) + str(self.accel.linear.y) + str(self.accel.linear.z)
        accelgyros += str(self.accel.angular.x) + str(self.accel.angular.y) + str(self.accel.angular.z)

        # write
        fmt = '<{}h'.format(len(accelgyros))
        data = struct.pack(fmt, *accelgyros)
        #header = '>DV'+str(device_id, 16)+str(len(data), 16)
        #self.serial.write_with_re_de(header+data)
        self.serial.write_with_re_de(data)

    def start(self):
        try:
            while not rospy.is_shutdown():
                self.write_accel()
                self.sleep_rate.sleep()
        finally:
            self.serial.close()


if __name__ == '__main__':
    Node().start()
