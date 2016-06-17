#!/usr/bin/env python
# coding=utf-8

# 便利な関数やメソッド
# - 文字列やバイト列の連結は join()
# - リストやタプルの各要素に関数を適用するときは map() または 内包表記
# - 1バイト整数のリストやタプルを bytes に変換するときは bytearray() して bytes()
# - bytes を数値列に変換するときは struct.unpack()
# - 連続した整数の用意は range() か xrange()

# 推奨される書式
# - 関数名は小文字とアンダースコア

# そのほか
# - 暗黙に仮定している (分かりにくい) 条件があれば assert文 で明示しておく


import struct

import mraa
import rospy
from std_msgs.msg import String


class Mpu(object):
    I2C_PORT = 0
    I2C_ADDRESS = 0x68

    def __init__(self):
        self._i2c = mraa.I2c(Mpu.I2C_PORT)
        self._i2c.address(Mpu.I2C_ADDRESS)
        self._i2c.writeReg(0x6B, 0x00)

    def read_accelgyros(self):
        start_address = 0x3B
        data_nbytes = 12
        reg_addresses = xrange(start_address, start_address + data_nbytes)

        # map(f, xs) は (f(x) for x in xs) と等価
        i2c_data = map(self._i2c.readReg, reg_addresses)

        # '>' :  ビッグエンディアン (上位バイトから先に送られてくる形式)
        # 'h' :  2byte符号付き整数
        data_format = '>' + 'h' * (data_nbytes / struct.calcsize('h'))
        assert struct.calcsize(data_format) == data_nbytes
        accelgyros = struct.unpack(data_format, bytes(bytearray(i2c_data)))

        return accelgyros


class Node(object):
    NAME = 'i2cNode'
    PUBLISHER_NAME = 'I2cToControl'
    SUBSCRIBER_NAME = 'ControlToI2c'
    ROSPY_RATE_HZ = 10

    def __init__(self):
        self.mpu = Mpu()

        rospy.init_node(Node.NAME, anonymous=True)
        self.publisher = rospy.Publisher(Node.PUBLISHER_NAME, String, queue_size=10)
        rospy.Subscriber(Node.SUBSCRIBER_NAME, String, self.subscribe)
        self.rospy_rate = rospy.Rate(Node.ROSPY_RATE_HZ)

    def subscribe(self, message):
        rospy.loginfo('controlNode %s', message.data)

        message_data = message.data.split(',')
        assert len(message_data) == 1

        rw, = message_data
        assert rw == 'r'

        self.publish_accelgyros()

    def publish_accelgyros(self):
        accelgyro = self.mpu.read_accelgyros()

        response = String()
        response.data = 'data,w,' + ','.join(map(str, accelgyro))
        self.publisher.publish(response)

    def start(self):
        try:
            while not rospy.is_shutdown():
                self.publish_accelgyros()
                self.rospy_rate.sleep()
        finally:
            pass


if __name__ == '__main__':
    Node().start()
