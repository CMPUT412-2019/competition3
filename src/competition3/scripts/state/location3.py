import os
import sys

from smach import State

sys.path.append(os.path.realpath(os.path.join(os.path.dirname(__file__), '../../../../')))

from src.line_follower.scripts.feature_detector import FeatureDetector, filter_by_distance, feature_depths
from src.competition3.scripts.util import notify_match


class Location3State(State):
    def __init__(self):
        State.__init__(self, outcomes=['ok', 'match', 'err'], input_keys=['green_shape'])
        self.feature_detector = FeatureDetector()

    def execute(self, ud):
        try:
            features = self.feature_detector.get_features()
            features = [f for f in features if f.colour == 'red']
            features = filter_by_distance(features, 1.)
            depths = feature_depths(features)
            feature = next((f for i, f in enumerate(features) if depths[i] == min(depths)), None)

            if feature is None:
                return 'err'

            print(feature.shape)

            if feature.shape == ud.green_shape:
                notify_match()
                return 'match'

            return 'ok'
        except Exception, e:
            print(e)
            return 'err'
