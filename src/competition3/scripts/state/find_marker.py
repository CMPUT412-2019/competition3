import rospy
from geometry_msgs.msg import Twist
from smach import State

import os
import sys

sys.path.append(os.path.realpath(os.path.join(os.path.dirname(__file__), '../../../../')))

from src.competition3.scripts.marker_tracker import MarkerTracker


class FindMarkerState(State):
    """
    Turns the rover in place until it sees an AR marker. Then returns the marker ID as userdata.

    Returned userdata
      - ID of the first AR marker to be seen
    """
    def __init__(self, marker_tracker, cmd_vel_topic):  # type: (MarkerTracker, str) -> None
        State.__init__(self, outcomes=['ok'], output_keys=['marker_id'])
        self.marker_tracker = marker_tracker
        self.twist_publisher = rospy.Publisher(cmd_vel_topic, Twist)

    def execute(self, ud):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            markers = self.marker_tracker.get_visible_markers()
            if markers:
                ud.marker_id = next(iter(markers))
                return 'ok'
            t = Twist()
            t.angular.z = 0.3
            self.twist_publisher.publish(t)
            rate.sleep()
