#!/usr/bin/env python
from __future__ import division, print_function

import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from image_geometry import PinholeCameraModel
from competition3.srv import CamPixelToPoint, CamPixelToPointResponse
from sensor_msgs.msg import CameraInfo, Image
from tf import TransformListener

from util import SubscriberValue


def np_from_poinstamped(pt):  # type: (PointStamped) -> np.ndarray
    return np.array([pt.point.x, pt.point.y, pt.point.z])


class CamPixelToPointServer:
    def __init__(self):
        self.camera_model = PinholeCameraModel()
        self.bridge = CvBridge()
        self.camera_model.fromCameraInfo(SubscriberValue('camera_info', CameraInfo).value)
        self.depth_image = SubscriberValue('camera_depth_image', Image, transform=self.bridge.imgmsg_to_cv2)
        self.service = rospy.Service('cam_pixel_to_point', CamPixelToPoint, self.handle_service)
        self.tf_listener = TransformListener()
        print('Service is ready.')

    def handle_service(self, req):  # type: (CamPixelToPoint) -> CamPixelToPointResponse
        x, y = int(req.cam_pixel.x), int(req.cam_pixel.y)
        methods = [self.read_depth_simple,
                   # self.read_depth_average,
                   self.read_depth_as_floor_depth]
        for method in methods:
            d = method(x, y)
            if not np.isnan(d):
                break

        pos = np.array(self.camera_model.projectPixelTo3dRay((x, y))) * d

        point = PointStamped()
        point.header.frame_id = self.camera_model.tfFrame()
        point.point.x, point.point.y, point.point.z = pos[0], pos[1], pos[2]
        return CamPixelToPointResponse(point)

    def read_depth_simple(self, x, y):  # (int, int) -> float
        return self.depth_image.value[y, x]

    def read_depth_average(self, x, y):  # (int, int) -> float
        print('Fallback to average')
        s = 5
        return np.nanmean(self.depth_image.value[y-s:y+s, x-s:x+s])

    def read_depth_as_floor_depth(self, x, y):  # (int, int) -> float
        print('Fallback to floor model')
        min_distance = 10.0
        # Extend the camera ray until it passes through where the floor should be. Use its length as the depth.
        camera_origin = PointStamped()
        camera_origin.header.frame_id = self.camera_model.tfFrame()
        camera_origin.point.x, camera_origin.point.y, camera_origin.point.z = 0.0, 0.0, 0.0
        point_along_ray = PointStamped()
        point_along_ray.header.frame_id = self.camera_model.tfFrame()
        point_along_ray.point.x, point_along_ray.point.y, point_along_ray.point.z = self.camera_model.projectPixelTo3dRay((x, y))

        self.tf_listener.waitForTransform('base_footprint', self.camera_model.tfFrame(), rospy.Time(rospy.get_time()), rospy.Duration(1))
        camera_origin = self.tf_listener.transformPoint('base_footprint', camera_origin)
        point_along_ray = self.tf_listener.transformPoint('base_footprint', point_along_ray)

        camera_origin = np_from_poinstamped(camera_origin)
        point_along_ray = np_from_poinstamped(point_along_ray)
        ray_dir = point_along_ray - camera_origin
        # Assuming this transformation was orthogonal, |ray_dir| = 1, at least approximately
        d = camera_origin[1]/max(-ray_dir[1], camera_origin[1]/min_distance)
        if d <= 0.01:
            d = np.nan
        return d


if __name__ == '__main__':
    rospy.init_node('cam_pixel_to_point')
    CamPixelToPointServer()
    rospy.spin()
