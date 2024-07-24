import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import message_filters
import numpy as np
import copy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped


class Camera(Node):
    def __init__(self, 
                 forward_camera_topic="/forward_camra",
                 forward_depth_topic="/forward_depth",
                 forward_depth_info_topic="/forward_depth_info",
                 forward_camera_link="/forward_camera_link",
                 slant_camera_topic="/slant_camera",
                 slant_depth_topic="/slant_depth",
                 slant_depth_info_topic="/slant_depth_info",
                 slant_camera_link="/slant_camera_link",
                 left_hand_camera_topic="/left_hand_camera",
                 left_hand_depth_topic="/left_hand_depth",
                 left_hand_depth_info_topic="/left_hand_depth_info",
                 left_hand_camera_link="/left_hand_camera_link", 
                 right_hand_camera_topic="/right_hand_camra",
                 right_hand_depth_topic="/right_hand_depth",
                 right_hand_depth_info_topic="/right_hand_depth_info",
                 right_hand_camera_link="/right_hand_camera_link",
                 base_link="/base_link"
                 ):
        super().__init__("camera_ndoe")
        self._forward_camera_info = None
        self._forward_color = None
        self._forward_depth = None
        self._slant_camera_info = None
        self._slant_color = None
        self._slant_depth = None
        self._left_hand_camera_info = None
        self._left_hand_color = None
        self._left_hand_depth = None
        self._right_hand_camera_info = None
        self._right_hand_color = None
        self._right_hand_depth = None
        self.forward_camera_link = forward_camera_link
        self.slant_camera_link = slant_camera_link
        self.left_hand_camera_link = left_hand_camera_link
        self.right_hand_camera_link = right_hand_camera_link
        self.base_link = base_link
        self.bridge = CvBridge()
        self.forward_camera_info_sub = self.create_subscription(CameraInfo, forward_depth_info_topic, self._forward_camera_info_cb, 10)
        self.forward_color_sub = self.create_subscription(Image, forward_camera_topic, self._forward_color_cb, 10)
        self.forward_depth_sub = self.create_subscription(Image, forward_depth_topic, self._forward_depth_cb, 10)
        self.slant_camera_info_sub = self.create_subscription(CameraInfo, slant_depth_info_topic, self._slant_camera_info_cb, 10)
        self.slant_color_sub = self.create_subscription(Image, slant_camera_topic, self._slant_color_cb, 10)
        self.slant_depth_sub = self.create_subscription(Image, slant_depth_topic, self._slant_depth_cb, 10)
        self.left_hand_camera_info_sub = self.create_subscription(CameraInfo, left_hand_depth_info_topic, self._left_hand_camera_info_cb, 10)
        self.left_hand_color_sub = self.create_subscription(Image, left_hand_camera_topic, self._left_hand_color_cb, 10)
        self.left_hand_depth_sub = self.create_subscription(Image, left_hand_depth_topic, self._left_hand_depth_cb, 10)
        self.right_hand_camera_info_sub = self.create_subscription(CameraInfo, right_hand_depth_info_topic, self._right_hand_camera_info_cb, 10)
        self.right_hand_color_sub = self.create_subscription(Image, right_hand_camera_topic, self._right_hand_color_cb, 10)
        self.right_hand_depth_sub = self.create_subscription(Image, right_hand_depth_topic, self._right_hand_depth_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    
    def _forward_camera_info_cb(self, camera_info: CameraInfo):
        if self._forward_camera_info is None:
            self._forward_camera_info = np.array(camera_info.k).reshape(3,3)
    
    def _forward_color_cb(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._forward_color = copy.deepcopy(cv_image)
    
    def _forward_depth_cb(self, msg: Image):
        cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        self._forward_depth = copy.deepcopy(cv_depth)
    
    def _slant_camera_info_cb(self, camera_info: CameraInfo):
        if self._slant_camera_info is None:
            self._slant_camera_info = np.array(camera_info.k).reshape(3,3)
    
    def _slant_color_cb(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._slant_color = copy.deepcopy(cv_image)
    
    def _slant_depth_cb(self, msg: Image):
        cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        self._slant_depth = copy.deepcopy(cv_depth)
    
    def _left_hand_camera_info_cb(self, camera_info: CameraInfo):
        if self._left_hand_camera_info is None:
            self._left_hand_camera_info = np.array(camera_info.k).reshape(3,3)
    
    def _left_hand_color_cb(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._left_hand_color = copy.deepcopy(cv_image)
    
    def _left_hand_depth_cb(self, msg: Image):
        cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        self._left_hand_depth = copy.deepcopy(cv_depth)
    
    def _right_hand_camera_info_cb(self, camera_info: CameraInfo):
        if self._right_hand_camera_info is None:
            self._right_hand_camera_info = np.array(camera_info.k).reshape(3,3)
    
    def _right_hand_color_cb(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._right_hand_color = copy.deepcopy(cv_image)
    
    def _right_hand_depth_cb(self, msg: Image):
        cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        self._right_hand_depth = copy.deepcopy(cv_depth)
    
    def get_camera_pose(self, camera_link, base_link):
        base_tf: TransformStamped = self.tf_buffer.lookup_transform(base_link, camera_link, self.get_clock().now())
        position = np.array([base_tf.transform.translation.x, base_tf.transform.translation.y, base_tf.transform.translation.z], dtype=np.float64)
        rotation = np.array([base_tf.transform.rotation.x, base_tf.transform.rotation.y, base_tf.transform.rotation.z, base_tf.transform.rotation.w])
        return (position, rotation)
    

    def get_forward_camera(self):
        return self._forward_color, self._forward_depth, self._forward_camera_info, self.get_camera_pose(self.forward_camera_link, self.base_link)
    
    def get_slant_camera(self):
        return self._slant_color, self._slant_depth, self._slant_camera_info, self.get_camera_pose(self.slant_camera_link, self.base_link)
    
    def get_left_hand_camera(self):
        return self._left_hand_color, self._left_hand_depth, self._left_hand_camera_info, self.get_camera_pose(self.left_hand_camera_link, self.base_link)
    
    def get_right_hand_camera(self):
        return self._right_hand_color, self._right_hand_depth, self._right_hand_camera_info, self.get_camera_pose(self.right_hand_camera_link, self.base_link)
