#!/usr/bin/env python

import rospy
from marker_tracker import MarkerTracker
from smach import Sequence
from smach_ros import IntrospectionServer
from state.find_marker import FindMarkerState
from state.navigate_to_marker import NavigateToMarkerState
from state.navigate_to_named_pose import NavigateToNamedPoseState


def main():
    rospy.init_node('ar_parking')
    marker_tracker = MarkerTracker()

    seq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok')
    with seq:
        Sequence.add('NavigateToStart', NavigateToNamedPoseState('ar_parking_start'))
        Sequence.add('FindMarker', FindMarkerState(marker_tracker, '/cmd_vel_mux/input/teleop'))
        Sequence.add('NavigateToMarker', NavigateToMarkerState(marker_tracker), transitions={'ok': 'NavigateToStart'})

    sis = IntrospectionServer('smach_server', seq, '/SM_ROOT')
    sis.start()

    seq.execute()


if __name__ == '__main__':
    main()
