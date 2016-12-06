#!/usr/bin/env python
import struct

import serial
import mraa
import rospy
from std_msgs.msg import String


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

        rospy.init_node('serialNode', anonymous=True)
        self.publisher = rospy.Publisher(
            'SerialToControl', String, queue_size=10)
        self.subscribers = (
            rospy.Subscriber('ControlToSerial', String,
                             self.subscribe_request),
            rospy.Subscriber('I2cToControl', String,
                             self.subscribe_accelgyros),
        )
        self.sleep_rate = rospy.Rate(self.SLEEP_RATE_HZ)

    def subscribe_request(self, message):
        request, text = message.data.split(',')
        if request == 'w':
            self.serial.write_with_re_de(text)
        else:
            rospy.logwarn('unknown request "%s" ignored.', request)

    def subscribe_accelgyros(self, message):
        data = message.data.split(',')
        self.accelgyros = map(int, data[2:])

    def write_accelgyros(self):
        # ready?
        if self.serial.inWaiting() <= 0:
            return
        data = self.serial.read()
        if data != '>':
            rospy.logwarn('invalid input')

        # write
        fmt = '>{}h'.format(len(self.accelgyros))
        text = struct.pack(fmt, *self.accelgyros)
        self.serial.write_with_re_de(text)

    def start(self):
        try:
            while not rospy.is_shutdown():
                self.write_accelgyros()
                self.sleep_rate.sleep()
        finally:
            self.serial.close()


if __name__ == '__main__':
    Node().start()
