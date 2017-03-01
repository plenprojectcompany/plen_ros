#!/usr/bin/env python
# coding=utf-8

__author__    = 'Tatsuroh SAKAGUCHI'
__author__    = 'Yugo KAJIWARA'
__copyright__ = 'PLEN Project Company, and all authors.'
__license__   = 'BSD'

import os
from glob import glob
import subprocess

import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage


def get_latest_modified_file_path(dirname):
    target = os.path.join(dirname, '*')
    files = [(f, os.path.getmtime(f)) for f in glob(target)]
    latest_modified_file_path = sorted(files, key=lambda files: files[1])[-1]
    return latest_modified_file_path[0]


class Node(object):
    SLEEP_RATE_HZ = 15

    def __init__(self):
        self.sequence = 0

        # register camera node to ROS
        rospy.init_node('camera_node', anonymous=True)

        # register publisher wanted to publish message of web camera
        self.stream_topic = rospy.Publisher('camera_stream', CompressedImage, queue_size=10)
        rospy.Subscriber('request_capture', Empty, self.subscribe)

        # run mjpg-streamer background
        rospy.loginfo('starting mjpg-streamer...')
        subprocess.call(['mkdir', '-p', '/tmp/ros/camera'])
        subprocess.Popen(['mjpg_streamer', '-i', '/usr/lib/input_uvc.so -d /dev/video0 -r 1280x720 -f 15', '-o',
                          '/usr/lib/output_file.so -f /tmp/ros/camera -s 10'])
        rospy.loginfo('done.')

        self.sleep_rate = rospy.Rate(self.SLEEP_RATE_HZ)

    def subscribe(self, message):
        rospy.loginfo('SUBSCRIBED REQUEST')
        file_path = get_latest_modified_file_path("/tmp/ros/camera/")

        header = Header()
        header.seq = self.sequence
        self.sequence += 1
        header.stamp.secs = int(os.path.getmtime(file_path))
        header.frame_id = "0"

        response = CompressedImage()
        response.format = "jpeg"
        f = open(file_path)
        response.data = f.read()
        f.close()
        response.header = header
        rospy.loginfo('PUBLISH JPG IMAGE')
        self.stream_topic.publish(response)

    def start(self):
        try:
            while not rospy.is_shutdown():
                self.sleep_rate.sleep()
        finally:
            rospy.loginfo('stopping mjpg-streamer...')
            subprocess.call(['killall', 'mjpg_streamer'])
            subprocess.call(['rm', '-r', '/tmp/ros/camera'])
            rospy.loginfo('done.')


if __name__ == '__main__':
    Node().start()
