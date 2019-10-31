#!/usr/bin/env python

from state_groups import WaitForJoyState
from state_groups import MoveToStopLineGroup, MoveToObstacleGroup, EnterSplitGroup, ExitSplitGroup
from state_groups import Location1Group, Location2Group, Location3Group
from state_groups import OffRampGroup, OnRampGroup, ArTagGroup, JoystickLocationGroup, FindShapeGroup

import rospy
from smach import Sequence
from smach_ros import IntrospectionServer


class UserData:
    def __init__(self):
        self.green_shape = None


def main():
    rospy.init_node('competition2')

    sq = Sequence(outcomes=['ok'], connector_outcome='ok')
    with sq:
        # Sequence.add('START', WaitForJoyState())

        Sequence.add('STOP1', MoveToStopLineGroup())
        Sequence.add('LOCATION1', Location1Group())

        Sequence.add('STOP2', MoveToStopLineGroup())

        Sequence.add('SPLIT_ENTER', EnterSplitGroup())

        Sequence.add('OBSTACLE1', MoveToObstacleGroup())
        Sequence.add('LOCATION2', Location2Group())

        Sequence.add('SPLIT_EXIT', ExitSplitGroup())

        Sequence.add('STOP3', MoveToStopLineGroup())

        Sequence.add('OFFRAMP', OffRampGroup())

        # Sequence.add('ARTAG', ArTagGroup())
        # Sequence.add('JOYLOC', JoystickLocationGroup())
        # Sequence.add('FINDSHAPE', FindShapeGroup())

        Sequence.add('ONRAMP', OnRampGroup())

        Sequence.add('STOP4', MoveToStopLineGroup(lower=True))

        Sequence.add('LOCATION3', Location3Group())

        Sequence.add('STOP5', MoveToStopLineGroup())

    sis = IntrospectionServer('smach_server', sq, '/SM_ROOT')
    sis.start()

    sq.execute()


if __name__ == '__main__':
    main()



