#!/usr/bin/env python
import rospy
import mraa
from std_msgs.msg import String


class PlenEye(object):

    def __init__(self, pin, pwm_period_us=700):
        self._pwm = mraa.Pwm(pin)
        self._pwm.period_us(pwm_period_us)
        self._pwm.enable(True)

    def on(self):
        self._pwm_duty = 1.0
        self._inc_speed = 0.0

    def off(self):
        self._pwm_duty = 0.0
        self._inc_speed = 0.0

    def act(self):
        self._pwm_duty = 0.0
        self._inc_speed = 0.05

    def update(self):
        self._pwm.write(self._pwm_duty)
        self._pwm_duty = max(0, min(1, self._pwm_duty + self._inc_speed))


def left_eye():
    return PlenEye(pin=14)


def right_eye():
    return PlenEye(pin=20)


class Node(object):
    ROSPY_RATE_HZ = 100

    def __init__(self):
        self.eyes = (left_eye(), right_eye())

        rospy.init_node('gpio_node', anonymous=True)
        self.subscriber = rospy.Subscriber(
            'to_gpio', String, self.subscribe)
        self.rospy_rate = rospy.Rate(self.ROSPY_RATE_HZ)

    def subscribe(self, message):
        rospy.loginfo("GPIO:%s", message.data)

        message_data = message.data.split(",")
        assert len(message_data) == 2

        rw, mode = message_data

        if rw != 'w':
            return

        for eye in self.eyes:
            if mode == "on":
                eye.on()
            elif mode == "off":
                eye.off()
            elif mode == "act":
                eye.act()
            else:
                assert False

    def start(self):
        try:
            while not rospy.is_shutdown():
                for eye in self.eyes:
                    eye.update()
                self.rospy_rate.sleep()
        finally:
            for eye in self.eyes:
                eye.off()


if __name__ == '__main__':
    Node().start()
