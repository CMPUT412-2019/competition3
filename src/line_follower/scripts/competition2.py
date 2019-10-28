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


class UserData:
    def __init__(self):
        self.green_shape = None


forward_speed = 0.4
kp = 4.
ki = 0.
kd = 0.
proximity_detector = ProximityDetector(1.)


def white_filter(hsv):  # type: (np.ndarray) -> np.ndarray
    return cv2.inRange(hsv, np.array([0, 0, 200]), np.array([255, 50, 255])).astype(bool)


def red_filter(hsv):  # type: (np.ndarray) -> np.ndarray
    f = (
            cv2.inRange(hsv, np.asarray([0, 70, 50]), np.asarray([10, 255, 250])) |
            cv2.inRange(hsv, np.asarray([170, 70, 50]), np.asarray([180, 255, 250]))
    ).astype(bool)
    # f[f.shape[0]-40:f.shape[0], :] = False
    return f


def lower_filter(hsv):    # type: (np.ndarray) -> np.ndarray
    f = np.zeros(hsv.shape[0:2], dtype=bool)
    f[f.shape[0]-40:f.shape[0]-20, :] = True
    return f


def red_lower_filter(hsv):    # type: (np.ndarray) -> np.ndarray
    return red_filter(hsv) & lower_filter(hsv)


def red_detector(hsv):   # type: (np.ndarray) -> np.ndarray
    return np.sum(red_filter(hsv)) > 300


def red_lower_detector(hsv):  # type: (np.ndarray) -> np.ndarray
    return np.sum(red_lower_filter(hsv)) > 300


rospy.init_node('competition2')

ud = UserData()

sq = Sequence(outcomes=['ok'], connector_outcome='ok')

with sq:
    # Go forward until location 1
    Sequence.add('FOLLOW1',  LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAfter(red_detector)))
    # Sequence.add('COAST1', ForwardState(forward_speed))

    # Location 1
    Sequence.add('TURN1', RotateState(np.pi / 2))
    Sequence.add('LOCATION1', Location1State())
    Sequence.add('TURN2', RotateState(-np.pi / 2))

    # Forward until split
    Sequence.add('FOLLOW2', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAfter(red_detector)))
    Sequence.add('STOP2', StopState(1))
    Sequence.add('FOLLOW3', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter,  TransitionAt(red_detector)))
    Sequence.add('FORWARD3', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), red_filter, TransitionAfter(red_detector)))
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
    Sequence.add('FOLLOW5', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAt(red_detector)))
    Sequence.add('FORWARD5', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), red_filter, TransitionAfter(red_detector)))
    # Sequence.add('COAST5', ForwardState(forward_speed))

    # Turn at split
    Sequence.add('TURN5', RotateState(np.pi/2))

    # Forward until location 3
    Sequence.add('FOLLOW6', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAfter(red_detector)))
    Sequence.add('STOP6', StopState(1))
    Sequence.add('FOLLOW7', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAfter(red_lower_detector)))
    Sequence.add('STOP7', StopState(1))

    # Location 3, part 1
    Sequence.add('FOLLOW8', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAfter(red_lower_detector)))
    Sequence.add('STOP8', ForwardState(forward_speed))
    Sequence.add('TURN8', RotateState(np.pi / 2))
    Sequence.add('LOCATION3_1', Location3State(ud))
    Sequence.add('TURN8_2', RotateState(-np.pi / 2))

    # Location 3, part 2
    Sequence.add('FOLLOW9', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAfter(red_lower_detector)))
    Sequence.add('STOP9', ForwardState(forward_speed))
    Sequence.add('TURN9', RotateState(np.pi / 2))
    Sequence.add('LOCATION3_2', Location3State(ud))
    Sequence.add('TURN9_2', RotateState(-np.pi / 2))

    # Location 3, part 3
    Sequence.add('FOLLOW10', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAfter(red_lower_detector)))
    Sequence.add('STOP10', ForwardState(forward_speed))
    Sequence.add('TURN10', RotateState(np.pi / 2))
    Sequence.add('LOCATION3_3', Location3State(ud))
    Sequence.add('TURN10_2', RotateState(-np.pi / 2))

    # Finish
    Sequence.add('FOLLOW11', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter, TransitionAfter(red_detector)))
    Sequence.add('STOP11', StopState())


sis = IntrospectionServer('smach_server', sq, '/SM_ROOT')
sis.start()

sq.execute()
