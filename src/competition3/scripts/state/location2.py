import os
import sys

from smach import State

sys.path.append(os.path.realpath(os.path.join(os.path.dirname(__file__), '../../../../')))

from src.line_follower.scripts.feature_detector import FeatureDetector
from src.line_follower.scripts.util import notify_count


class Location2State(State):
    def __init__(self):
        State.__init__(self, outcomes=['ok', 'err'], output_keys=['green_shape'])
        self.feature_detector = FeatureDetector()

    def execute(self, ud):
        try:
            features = self.feature_detector.get_features()
            green_shape = next(f.shape for f in features if f.colour == 'green')
            ud.green_shape = green_shape
            notify_count(len(features))
            print(ud.green_shape)
            return 'ok'
        except Exception:
            ud.green_shape = None
            return 'err'
