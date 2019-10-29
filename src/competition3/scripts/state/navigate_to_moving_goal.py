import rospy
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from smach import State
from typing import Callable


class NavigateToMovingGoalState(State):
    """
    Moves the robot to a moving goal in the map frame.
    """
    def __init__(self, goal):  # type: (Callable[[], PoseStamped]) -> None
        State.__init__(self, outcomes=['ok', 'err'])
        self.goal = goal
        self.client = SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, ud):
        self.client.wait_for_server()
        while True:
            pose = self.goal()
            if pose is None:
                return 'err'

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position = pose.pose.position
            goal.target_pose.pose.orientation = pose.pose.orientation
            self.client.send_goal(goal)
            if self.client.wait_for_result(timeout=rospy.Duration(1)):
                break

        return 'ok'
