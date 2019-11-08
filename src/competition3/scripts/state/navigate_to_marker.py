import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from smach import State
from tf.transformations import quaternion_conjugate, quaternion_multiply, unit_vector

from src.competition3.scripts.marker_tracker import MarkerTracker
from src.competition3.scripts.state.navigate_to_moving_goal import NavigateToMovingGoalState


def qv_mult(q1, v1):
    # https: // answers.ros.org / question / 196149 / how - to - rotate - vector - by - quaternion - in -python /
    v1 = unit_vector(v1)
    q2 = list(v1) + [0.0]
    return quaternion_multiply(quaternion_multiply(q1, q2), quaternion_conjugate(q1))[:3]


class NavigateToMarkerState(State):
    """
    Moves the robot to 0.5 meters in front of the last known position of the AR tag of a given id. This position is
    updated by a MarkerTracker.

    Required userdata
      - marker_id (int): The ID of the marker to look for.
    """
    def __init__(self, marker_tracker):  # type: (MarkerTracker) -> None
        State.__init__(self, outcomes=['ok', 'err'], input_keys=['marker_id'])
        self.marker_tracker = marker_tracker

    def execute(self, ud):

        def get_pose():
            marker_pose = self.marker_tracker.get_pose(ud.marker_id)
            if marker_pose is None:
                return marker_pose

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = goal_pose.header.frame_id

            marker_orientation = marker_pose.pose.orientation
            marker_orientation = np.array([marker_orientation.x, marker_orientation.y, marker_orientation.z, marker_orientation.w])
            offset = qv_mult(marker_orientation, [0, 0, 1]) * 0.4
            goal_position = Point(marker_pose.pose.position.x + offset[0],
                             marker_pose.pose.position.y + offset[1],
                             marker_pose.pose.position.z + offset[2])
            goal_pose.pose.position = goal_position
            goal_pose.pose.orientation.w = 1.0
            return goal_pose

        return NavigateToMovingGoalState(get_pose).execute({})
