#!/usr/bin/env python

import cv2
import cv_bridge
import rospy

from feature_detector import FeatureDetector, filter_by_distance, select_center
from sensor_msgs.msg import Image
import numpy as np


def main():
    rospy.init_node('shape_detect')
    bridge = cv_bridge.CvBridge()
    feature_detector = FeatureDetector()
    main.image = None

    def image_callback(msg):
        main.image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)

    while not rospy.is_shutdown():
        image = main.image
        if image is None:
            continue

        rospy.sleep(1)
        image = main.image



        try:
            features = feature_detector.get_features()
            # features = filter_by_distance(features, 1.0)
            # features = [select_center(features)]
        except Exception as err:
            print(err)
            raise
        print(len(features))
        for index, feature in enumerate(features):
            if index%2:
                offset = np.array([0, 30])
            else:
                offset = np.array([0, -30])
            print(feature.centroid+offset)
            image = cv2.putText(
                image,
                '{}'.format(feature.shape),
                tuple(int(x) for x in feature.centroid+offset),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                FeatureDetector.col_name_to_rgb(feature.colour),
            )
            col = FeatureDetector.col_name_to_rgb(feature.colour)
            cv2.drawContours(image, [feature.contour], -1, col, 5)

        cv2.imshow('image', image)
        # cv2.imshow('hsv', cv2.cvtColor(image, cv2.COLOR_BGR2HSV))
        cv2.waitKey(0)

    rospy.spin()

    # from glob import glob
    #
    # filenames = glob('images/**.*')
    #
    # while not rospy.is_shutdown():
    #     for filename in filenames:
    #         image = cv2.imread(filename)
    #         w = 640
    #         h = 640*image.shape[1]/image.shape[0]
    #         image = cv2.resize(image, (h, w))
    #         msg = bridge.cv2_to_imgmsg(image, encoding='bgr8')
    #         rate = rospy.Rate(10)
    #         start_time = rospy.get_time()
    #         while not rospy.is_shutdown():
    #             image_callback(msg)
    #             if rospy.get_time() - start_time > 2:
    #                 break
    #         rate.sleep()


if __name__ == '__main__':
    main()
