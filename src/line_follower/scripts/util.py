import rospy
from typing import Any, Callable, Optional
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import Led
import playsound
import numpy as np
import time
from os import path


class SubscriberValue:
    def __init__(self, name, data_class, wait=True, queue_size=1, transform=None):  # type: (str, Any, bool, int, Optional[Callable[[Any], Any]]) -> None
        self._subscriber = rospy.Subscriber(name, data_class, callback=self._callback, queue_size=queue_size)
        self._topic = name
        self._wait = wait
        self._transform = transform
        self._value = None

    def _callback(self, message):
        if self._transform is None:
            self._value = message
        else:
            self._value = self._transform(message)

    def wait(self):
        while self._value is None and not rospy.is_shutdown():
            rospy.loginfo('Waiting for {}...'.format(self._topic))
            rospy.sleep(0.1)
        return self._value

    @property
    def value(self):
        if self._wait:
            self.wait()
        return self._value


class ProximityDetector:
    def __init__(self, proximity=1):
        self.proximity = proximity
        self.laser_scan = SubscriberValue('scan', LaserScan)

    def __call__(self, _):
        return np.nanmin(self.laser_scan.value.ranges) < self.proximity


def led(msg):  # type: (str) -> None
    msg = msg.upper()

    led_pub = {1: rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1, latch=True),
               2: rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1, latch=True)}
    led_col = {'R': Led.RED, 'G': Led.GREEN, 'O': Led.ORANGE, 'B': Led.BLACK}

    i = 0
    while i < len(msg):
        if msg[i] in led_col.keys() and i + 1 < len(msg):
            led_pub[int(msg[i+1])].publish(led_col[msg[i]])
            i = i + 1
        elif msg[i] == 'W':
            time.sleep(2)
        else:
            raise ValueError('Invalid message character: {}'.format(msg[i]))
        i = i + 1


def notify_count(count):
    led(['b1b2', 'b1b2', 'g1b2', 'g1g2'][count])
    playsound.playsound(path.join(path.dirname(__file__), '../../../sound/{}.mp3'.format(count)), block=True)
    led('b1b2')


def notify_match():
    led('r1r2')
    playsound.playsound(path.join(path.dirname(__file__), '../../../sound/match.mp3'), block=True)
    led('b1b2')
