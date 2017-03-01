#!/usr/bin/env python
# coding=utf-8

__author__    = 'Tatsuroh SAKAGUCHI'
__author__    = 'Yugo KAJIWARA'
__copyright__ = 'PLEN Project Company, and all authors.'
__license__   = 'BSD'

import subprocess

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Empty


class Node(object):
    SLEEP_RATE_HZ = 15

    def __init__(self):

        # register camera node to ROS
        rospy.init_node('receive_image_node', anonymous=True)

        # register publisher wanted to publish message of web camera
        self.stream_topic = rospy.Publisher('request_capture', Empty, queue_size=10)
        self.subscriber = rospy.Subscriber('camera_stream', CompressedImage, self.subscribe)

        self.sleep_rate = rospy.Rate(self.SLEEP_RATE_HZ)

    def subscribe(self, message):
        rospy.loginfo('RECEIVED IMAGE')
        f = open('/tmp/ros/camera/received_image.jpg', 'w')
        f.write(message.data)
        f.close()

    def start(self):
        try:
            while not rospy.is_shutdown():
                self.stream_topic.publish(Empty())
                self.sleep_rate.sleep()
        finally:
            subprocess.call(['rm', '/tmp/ros/camera/receive_image.jpg'])


if __name__ == '__main__':
    Node().start()
