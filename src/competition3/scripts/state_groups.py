from state.find_marker import FindMarkerState, MarkerTracker
from state.navigate_to_marker import NavigateToMarkerState
from state.navigate_to_named_pose import NavigateToNamedPoseState
from state.linefollow import LineFollowState, TransitionAfter, TransitionAt
from state.stop import StopState
from state.rotate import RotateState
from state.location1 import Location1State
from state.location2 import Location2State
from state.location3 import Location3State
from state.navigate_to_number import NavigateToNumberState
from state.select_number import SelectNumberState, JoystickInput
from pid_control import PIDController
from util import ProximityDetector, notify_match, notify_artag, notify_unmarked
from filter import RedDetector, RedLowerDetector, white_filter, red_filter

import rospy
from smach import State, Sequence, StateMachine

import numpy as np


forward_speed = 0.8
kp = 4.
ki = 0.
kd = 0.
proximity_detector = ProximityDetector(1.)


class WaitForJoyState(State):
    def __init__(self):
        super(WaitForJoyState, self).__init__(outcomes=['ok'])
        self.joystick = JoystickInput()

    def execute(self, ud):
        while not rospy.is_shutdown():
            button = self.joystick.wait_for_press()
            if button == 'A':
                return 'ok'


class Location1Group(State):
    def __init__(self):
        super(Location1Group, self).__init__(outcomes=['ok'])

    def execute(self, ud):
        sq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok')
        with sq:
            Sequence.add('TURN_L', RotateState(np.pi / 2))
            Sequence.add('LOCATION1', Location1State(), transitions={'err': 'TURN_R'})
            Sequence.add('TURN_R', RotateState(-np.pi / 2))
        return sq.execute(ud)


class MoveToStopLineGroup(State):
    def __init__(self, lower=False):
        super(MoveToStopLineGroup, self).__init__(outcomes=['ok'])
        self.detector = RedDetector() if not lower else RedLowerDetector()

    def execute(self, ud):
        sq = Sequence(outcomes=['ok'], connector_outcome='ok')
        with sq:
            # Forward until stop line
            Sequence.add('FOLLOW', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter,
                                                   TransitionAfter(self.detector)))
            Sequence.add('STOP', StopState())

        return sq.execute(ud)


class EnterSplitGroup(State):
    def __init__(self):
        super(EnterSplitGroup, self).__init__(outcomes=['ok'])

    def execute(self, ud):
        sq = Sequence(outcomes=['ok'], connector_outcome='ok')
        with sq:
            Sequence.add('FOLLOW_W', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter,
                                                     TransitionAt(RedDetector())))
            Sequence.add('FOLLOW_R', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), red_filter,
                                                     TransitionAfter(RedDetector())))
            Sequence.add('STOP', StopState())
            Sequence.add('TURN', RotateState(np.pi / 2 * 2 / 3))
        return sq.execute(ud)


class MoveToObstacleGroup(State):
    def __init__(self):
        super(MoveToObstacleGroup, self).__init__(outcomes=['ok'])

    def execute(self, ud):
        sq = Sequence(outcomes=['ok'], connector_outcome='ok')
        with sq:
            Sequence.add('FOLLOW', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter,
                                                    TransitionAt(proximity_detector)))
            Sequence.add('STOP', StopState())
        return sq.execute(ud)


class Location2Group(State):
    def __init__(self):
        super(Location2Group, self).__init__(outcomes=['ok'], output_keys=['green_shape'])

    def execute(self, ud):
        sq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok', output_keys=['green_shape'])
        with sq:
            Sequence.add('LOCATION2', Location2State(), transitions={'err': 'TURN'})
            Sequence.add('TURN', RotateState(np.pi))
        return sq.execute(ud)


class ExitSplitGroup(State):
    def __init__(self):
        super(ExitSplitGroup, self).__init__(outcomes=['ok'])

    def execute(self, ud):
        sq = Sequence(outcomes=['ok'], connector_outcome='ok')
        with sq:
            Sequence.add('FOLLOW_W', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), white_filter,
                                                     TransitionAt(RedDetector())))
            Sequence.add('FOLLOW_R', LineFollowState(forward_speed, PIDController(kp=kp, ki=ki, kd=kd), red_filter,
                                                     TransitionAfter(RedDetector())))
            Sequence.add('TURN', RotateState(np.pi / 2))
        return sq.execute(ud)


class OffRampGroup(State):
    def __init__(self):
        super(OffRampGroup, self).__init__(outcomes=['ok'])

    def execute(self, ud):
        sq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok')
        with sq:
            Sequence.add('GOAL_OFFRAMP_START', NavigateToNamedPoseState('off_ramp_start'), transitions={'err': 'GOAL_OFFRAMP_END'})
            Sequence.add('GOAL_OFFRAMP_END', NavigateToNamedPoseState('off_ramp_end'))
        sq.execute(ud)
        return 'ok'


class OnRampGroup(State):
    def __init__(self):
        super(OnRampGroup, self).__init__(outcomes=['ok'])

    def execute(self, ud):
        sq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok')
        with sq:
            Sequence.add('GOAL_ONRAMP', NavigateToNamedPoseState('on_ramp'))
        sq.execute(ud)
        return 'ok'


class ArTagGroup(State):
    def __init__(self):
        super(ArTagGroup, self).__init__(outcomes=['ok'])

    def execute(self, ud):
        marker_tracker = MarkerTracker()
        sq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok')
        with sq:
            Sequence.add('AR_FIND', FindMarkerState(marker_tracker, 'cmd_vel_mux/input/teleop'))
            Sequence.add('AR_GOTO', NavigateToMarkerState(marker_tracker))
        result = sq.execute(ud)
        if result == 'ok':
            notify_artag()
        return 'ok'


class JoystickLocationGroup(State):
    def __init__(self):
        super(JoystickLocationGroup, self).__init__(outcomes=['ok'])

    def execute(self, ud):
        sq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok')
        with sq:
            Sequence.add('SELECT_NUMBER', SelectNumberState(min=1, max=8))
            Sequence.add('GOAL', NavigateToNumberState())
        outcome = sq.execute(ud)
        if outcome == 'ok':
            notify_unmarked()
        return 'ok'


class Location3Group(State):
    def __init__(self):
        super(Location3Group, self).__init__(outcomes=['ok'], input_keys=['green_shape'])

    def execute(self, ud):
        sm = StateMachine(outcomes=['ok', 'match'], input_keys=['green_shape'])
        with sm:
            StateMachine.add('MOVETO1', MoveToStopLineGroup(lower=True), transitions={'ok': 'TURN1_1'})
            StateMachine.add('TURN1_1', RotateState(np.pi / 2), transitions={'ok': 'LOCATION3_1'})
            StateMachine.add('LOCATION3_1', Location3State(), transitions={'ok': 'TURN1_2', 'err': 'TURN1_2', 'match': 'EXIT_TURN1'})
            StateMachine.add('TURN1_2', RotateState(-np.pi / 2), transitions={'ok': 'MOVETO2'})

            StateMachine.add('MOVETO2', MoveToStopLineGroup(lower=True), transitions={'ok': 'TURN2_1'})
            StateMachine.add('TURN2_1', RotateState(np.pi / 2), transitions={'ok': 'LOCATION3_2'})
            StateMachine.add('LOCATION3_2', Location3State(), transitions={'ok': 'TURN2_2', 'err': 'TURN2_2', 'match': 'EXIT_TURN2'})
            StateMachine.add('TURN2_2', RotateState(-np.pi / 2), transitions={'ok': 'MOVETO3'})

            StateMachine.add('MOVETO3', MoveToStopLineGroup(lower=True), transitions={'ok': 'TURN3_1'})
            StateMachine.add('TURN3_1', RotateState(np.pi / 2), transitions={'ok': 'LOCATION3_3'})
            StateMachine.add('LOCATION3_3', Location3State(), transitions={'ok': 'TURN3_2', 'err': 'TURN3_2', 'match': 'EXIT_TURN3'})
            StateMachine.add('TURN3_2', RotateState(-np.pi / 2))

            StateMachine.add('EXIT_TURN1', RotateState(-np.pi / 2), transitions={'ok': 'EXIT_MOVETO2'})
            StateMachine.add('EXIT_TURN2', RotateState(-np.pi / 2), transitions={'ok': 'EXIT_MOVETO3'})
            StateMachine.add('EXIT_TURN3', RotateState(-np.pi / 2))
            StateMachine.add('EXIT_MOVETO2', MoveToStopLineGroup(lower=True), transitions={'ok': 'EXIT_MOVETO3'})
            StateMachine.add('EXIT_MOVETO3', MoveToStopLineGroup(lower=True))
        sm.execute(ud)
        return 'ok'


class FindShapeGroup(State):
    def __init__(self):
        super(FindShapeGroup, self).__init__(outcomes=['ok'], input_keys=['green_shape'])

    def execute(self, ud):
        sq = Sequence(outcomes=['ok', 'err', 'match'], connector_outcome='ok', input_keys=['green_shape'])
        with sq:
            Sequence.add('GOAL_1', NavigateToNamedPoseState('S1'))
            Sequence.add('FIND_1', Location3State(), transitions={'err': 'GOAL_2'})
            Sequence.add('GOAL_2', NavigateToNamedPoseState('S2'))
            Sequence.add('FIND_2', Location3State(), transitions={'err': 'GOAL_3'})
            Sequence.add('GOAL_3', NavigateToNamedPoseState('S3'))
            Sequence.add('FIND_3', Location3State(), transitions={'err': 'GOAL_4'})
            Sequence.add('GOAL_4', NavigateToNamedPoseState('S4'))
            Sequence.add('FIND_4', Location3State(), transitions={'err': 'GOAL_5'})
            Sequence.add('GOAL_5', NavigateToNamedPoseState('S5'))
            Sequence.add('FIND_5', Location3State(), transitions={'err': 'GOAL_6'})
            Sequence.add('GOAL_6', NavigateToNamedPoseState('S6'))
            Sequence.add('FIND_6', Location3State(), transitions={'err': 'GOAL_7'})
            Sequence.add('GOAL_7', NavigateToNamedPoseState('S7'))
            Sequence.add('FIND_7', Location3State(), transitions={'err': 'GOAL_8'})
            Sequence.add('GOAL_8', NavigateToNamedPoseState('S8'))
            Sequence.add('FIND_8', Location3State())
        sq.execute(ud)
        return 'ok'
