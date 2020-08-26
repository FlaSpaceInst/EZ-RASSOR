#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, CameraInfo
import numpy as np
import image_geometry


class PointCloudProcessor(object):
    """Coordinate System"""

    XYZ = {"RIGHT": 0, "DOWN": 1, "FORWARD": 2}

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.point_cloud = None
        camera_info = rospy.wait_for_message("depth/camera_info", CameraInfo)
        self.init_pc_info(camera_info)
        rospy.Subscriber("depth/points", PointCloud2, self.on_pc_update)

    """Stores the most recent PointCloud2 message received"""

    def on_pc_update(self, pc):
        self.point_cloud = pc

    """Converts PointCloud2 to numpy array

    Reading the most recent PointCloud2 message received, this method creates and
    returns a numpy array with all the non-NaN points in the point cloud. If the
    point cloud is entirely NaN values, None is returned.
    """

    def get_points(self):
        if self.point_cloud is None:
            return None

        # Read points directly from point cloud message
        pc = np.frombuffer(self.point_cloud.data, np.float32)
        # Reshape into array of xyz values
        pc = np.reshape(pc, (-1, 8))[:, :3]
        # Remove nan points
        pc = pc[~np.isnan(pc).any(axis=1)]

        return pc if pc.size > 0 else None

    """Returns the angle (in radians) between two 3D rays"""

    @staticmethod
    def angle_between_rays(ray1, ray2):
        dot_product = ray1[0] * ray2[0] + ray1[1] * ray2[1] + ray1[2] * ray2[2]
        magnitude1 = np.sqrt(((ray1[0] ** 2) + (ray1[1] ** 2) + (ray1[2] ** 2)))
        magnitude2 = np.sqrt(((ray2[0] ** 2) + (ray2[1] ** 2) + (ray2[2] ** 2)))
        return np.arccos(dot_product / (magnitude1 * magnitude2))

    """Initializes information about point cloud based on CameraInfo message

    Analyzes a given CameraInfo message to find the minimum and maximum angles
    the camera can see. Subclasses can extend this to get information specific
    to their application. This method, as well as the "angle_between_rays"
    method, is based heavily on the C++ source code of the
    "depthimage_to_laserscan" package.
    """

    def init_pc_info(self, camera_info):
        # Initialize camera
        cam_model = image_geometry.PinholeCameraModel()
        cam_model.fromCameraInfo(camera_info)
        width = camera_info.width

        # Find vector for leftmost view in camera
        raw_pixel_left = (0, cam_model.cy())
        rect_pixel_left = cam_model.rectifyPoint(raw_pixel_left)
        left_ray = cam_model.projectPixelTo3dRay(rect_pixel_left)

        # Find vector for rightmost view in camera
        raw_pixel_right = (width - 1, cam_model.cy())
        rect_pixel_right = cam_model.rectifyPoint(raw_pixel_right)
        right_ray = cam_model.projectPixelTo3dRay(rect_pixel_right)

        # Find vector for center view of camera
        raw_pixel_center = (cam_model.cx(), cam_model.cy())
        rect_pixel_center = cam_model.rectifyPoint(raw_pixel_center)
        center_ray = cam_model.projectPixelTo3dRay(rect_pixel_center)

        # Find the range of angles to be covered by the point cloud
        self.angle_max = PointCloudProcessor.angle_between_rays(
            left_ray, center_ray
        )
        self.angle_min = -PointCloudProcessor.angle_between_rays(
            center_ray, right_ray
        )
