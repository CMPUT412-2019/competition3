import os
import sys

from smach import State

sys.path.append(os.path.realpath(os.path.join(os.path.dirname(__file__), '../../../../')))

from src.line_follower.scripts.feature_detector import FeatureDetector, filter_by_distance
from src.line_follower.scripts.util import notify_count


class Location1State(State):
    def __init__(self):
        State.__init__(self, outcomes=['ok', 'err'])
        self.feature_detector = FeatureDetector()

    def execute(self, ud):
        try:
            features = self.feature_detector.get_features()
            features = [f for f in features if f.colour == 'red']
            features = filter_by_distance(features, max_distance=1.)
            notify_count(len(features))
            return 'ok'
        except Exception:
            return 'err'