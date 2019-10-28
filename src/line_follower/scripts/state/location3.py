import os
import sys

from smach import State

sys.path.append(os.path.realpath(os.path.join(os.path.dirname(__file__), '../../../../')))

from src.line_follower.scripts.feature_detector import FeatureDetector, filter_by_distance, feature_depths
from src.line_follower.scripts.util import notify_match


class Location3State(State):
    def __init__(self, ud):
        State.__init__(self, outcomes=['ok'])
        self.feature_detector = FeatureDetector()
        self.ud = ud

    def execute(self, ud):
        features = self.feature_detector.get_features()
        features = [f for f in features if f.colour == 'red']
        features = filter_by_distance(features, 1.)
        depths = feature_depths(features)
        feature = next(f for i, f in enumerate(features) if depths[i] == min(depths))
        # feature = select_center(features)
        if feature.shape == self.ud.green_shape:
            notify_match()

        print(feature.shape)

        return 'ok'
