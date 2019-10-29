#!/usr/bin/env python

from state.linefollow import LineFollowState, TransitionAfter, TransitionAt
from state.stop import StopState
from state.rotate import RotateState
from state.forward import ForwardState
from state.location1 import Location1State
from state.location2 import Location2State
from state.location3 import Location3State
from pid_control import PIDController
from util import ProximityDetector

import rospy
import numpy as np
import cv2
from smach import Sequence
from smach_ros import IntrospectionServer
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class UserData:
    def __init__(self):
        self.green_shape = None


forward_speed = 0.4
kp = 4.
ki = 0.
kd = 0.
proximity_detector = ProximityDetector(1.)


def white_filter(hsv):  # type: (np.ndarray) -> np.ndarray
    return cv2.inRange(hsv, np.array([0, 0, 200]), np.array([255, 70, 255])).astype(bool)


def red_filter(hsv):  # type: (np.ndarray) -> np.ndarray
    f = (
            cv2.inRange(hsv, np.asarray([0, 70, 50]), np.asarray([10, 255, 255])) |
            cv2.inRange(hsv, np.asarray([160, 70, 50]), np.asarray([180, 255, 255]))
    ).astype(bool)
    # f[f.shape[0]-40:f.shape[0], :] = False
    return f


def lower_filter(hsv):    # type: (np.ndarray) -> np.ndarray
    f = np.zeros(hsv.shape[0:2], dtype=bool)
    f[f.shape[0]-40:f.shape[0]-20, :] = True
    return f


def red_lower_filter(hsv):    # type: (np.ndarray) -> np.ndarray
    return red_filter(hsv) & lower_filter(hsv)


class Detector(object):
    def __init__(self, filter, threshold, gui_topic):
        self.gui_filter_pub = rospy.Publisher(gui_topic, Image, queue_size=1)
        self.cv_bridge = CvBridge()
        self.filter = filter
        self.threshold = threshold

    def __call__(self, hsv):   # type: (np.ndarray) -> np.ndarray
        I = self.filter(hsv)
        self.gui_filter_pub.publish(self.cv_bridge.cv2_to_imgmsg(I.astype('uint8') * 255))
        return np.sum(I) > self.threshold


class RedDetector(Detector):
    def __init__(self):
        super(RedDetector, self).__init__(red_filter, 3000, 'gui/red_filter')


class RedLowerDetector(Detector):
    def __init__(self):
        super(RedLowerDetector, self).__init__(red_lower_filter, 600, 'gui/red_filter')


rospy.init_node('competition2')

ud = UserData()

sq = Sequence(outcomes=['ok'], connector_outcome='ok')

with sq:
    # Go forward until location 1
    Sequence.add('FOLLOW1',  LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAfter(RedDetector())))
    # Sequence.add('COAST1', ForwardState(forward_speed))

    # Location 1
    Sequence.add('TURN1', RotateState(np.pi / 2))
    Sequence.add('LOCATION1', Location1State())
    Sequence.add('TURN2', RotateState(-np.pi / 2))

    # Forward until split
    Sequence.add('FOLLOW2', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAfter(RedDetector())))
    Sequence.add('STOP2', StopState(1))
    Sequence.add('FOLLOW3', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter,  TransitionAt(RedDetector())))
    Sequence.add('FORWARD3', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), red_filter, TransitionAfter(RedDetector())))
    Sequence.add('COAST3', StopState())

    # Turn at split
    Sequence.add('TURN3', RotateState(np.pi/4))

    # Forward until location 2
    Sequence.add('FOLLOW4', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAt(proximity_detector)))

    # Location 2
    Sequence.add('STOP4', StopState(1))
    Sequence.add('LOCATION2', Location2State(ud))
    Sequence.add('TURN4', RotateState(np.pi))

    # Forward until split
    Sequence.add('FOLLOW5', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAt(RedDetector())))
    Sequence.add('FORWARD5', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), red_filter, TransitionAfter(RedDetector())))
    # Sequence.add('COAST5', ForwardState(forward_speed))

    # Turn at split
    Sequence.add('TURN5', RotateState(np.pi/2))

    # Forward until location 3
    Sequence.add('FOLLOW6', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAfter(RedDetector())))
    Sequence.add('STOP6', StopState(1))
    Sequence.add('FOLLOW7', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAfter(RedLowerDetector())))
    Sequence.add('STOP7', StopState(1))

    # Location 3, part 1
    Sequence.add('FOLLOW8', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAfter(RedLowerDetector())))
    Sequence.add('STOP8', ForwardState(forward_speed))
    Sequence.add('TURN8', RotateState(np.pi / 2))
    Sequence.add('LOCATION3_1', Location3State(ud))
    Sequence.add('TURN8_2', RotateState(-np.pi / 2))

    # Location 3, part 2
    Sequence.add('FOLLOW9', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAfter(RedLowerDetector())))
    Sequence.add('STOP9', ForwardState(forward_speed))
    Sequence.add('TURN9', RotateState(np.pi / 2))
    Sequence.add('LOCATION3_2', Location3State(ud))
    Sequence.add('TURN9_2', RotateState(-np.pi / 2))

    # Location 3, part 3
    Sequence.add('FOLLOW10', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAfter(RedLowerDetector())))
    Sequence.add('STOP10', ForwardState(forward_speed))
    Sequence.add('TURN10', RotateState(np.pi / 2))
    Sequence.add('LOCATION3_3', Location3State(ud))
    Sequence.add('TURN10_2', RotateState(-np.pi / 2))

    # Finish
    Sequence.add('FOLLOW11', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAfter(RedDetector())))
    Sequence.add('STOP11', StopState())


sis = IntrospectionServer('smach_server', sq, '/SM_ROOT')
sis.start()

sq.execute()
