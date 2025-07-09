#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

class DualCameraStitcher(Node):
    def __init__(self):
        super().__init__('dual_camera_stitcher')
        
        self.bridge = CvBridge()
        
        # Load homography matrix
        try:
            self.H = np.load("dual_camera_calibration/calibration_images/H_right_to_left.npy")  # Homography from right to left
            self.get_logger().info("成功加载单应矩阵")
        except FileNotFoundError:
            self.get_logger().error("未找到单应矩阵文件: dual_camera_calibration/H_right_to_left.npy")
            raise
        
        # Image storage
        self.img_left = None
        self.img_right = None
        
        # Flat field correction
        self.declare_parameter('use_flat_field', False)
        self.use_flat_field = self.get_parameter('use_flat_field').get_parameter_value().bool_value
        self.flat_left = None
        self.flat_right = None
        
        if self.use_flat_field:
            try:
                self.flat_left = np.load("dual_camera_flat_field/flat_left.npy")
                self.flat_right = np.load("dual_camera_flat_field/flat_right.npy")
                self.get_logger().info("启用平场校正")
            except FileNotFoundError:
                self.get_logger().error("未找到平场校正文件")
                self.use_flat_field = False
        else:
            self.get_logger().warn("未启用平场校正")
        
        # QoS configuration
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Subscribers
        self.left_sub = self.create_subscription(
            Image,
            "/hk_camera/left_camera/image_raw",
            self.callback_left,
            qos_profile
        )
        
        self.right_sub = self.create_subscription(
            Image,
            "/hk_camera/right_camera/image_raw",
            self.callback_right,
            qos_profile
        )
        
        # Publishers
        self.pub_stitched = self.create_publisher(Image, "/stitched_image", qos_profile)
        self.pub_debug = self.create_publisher(Image, "/debug_image", qos_profile)
        
        # Timer for processing
        self.timer = self.create_timer(1.0/60.0, self.process_images)  # 60 Hz
        
        self.get_logger().info("双相机拼接节点已启动")

    def correct_flat(self, image, flat_map, epsilon=1e-6):
        """平场校正函数"""
        image_f32 = image.astype(np.float32)
        flat_safe = np.where(flat_map < epsilon, epsilon, flat_map)

        if len(image.shape) == 3:
            for c in range(3):
                image_f32[:, :, c] = image_f32[:, :, c] / flat_safe
        else:
            image_f32 = image_f32 / flat_safe

        return np.clip(image_f32, 0, 255).astype(np.uint8)

    def callback_left(self, msg):
        """左相机图像回调"""
        img_left_raw = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.use_flat_field and self.flat_left is not None:
            self.img_left = self.correct_flat(img_left_raw, self.flat_left)
        else:
            self.img_left = img_left_raw

    def callback_right(self, msg):
        """右相机图像回调"""
        img_right_raw = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.use_flat_field and self.flat_right is not None:
            self.img_right = self.correct_flat(img_right_raw, self.flat_right)
        else:
            self.img_right = img_right_raw

    def warp_and_stitch(self, img_left, img_right, H):
        """图像变换和拼接"""
        h_left, w_left = img_left.shape[:2]
        h_right, w_right = img_right.shape[:2]

        corners_right = np.array([[0, 0], [w_right, 0], [w_right, h_right], [0, h_right]], dtype=np.float32).reshape(-1, 1, 2)
        corners_right_trans = cv2.perspectiveTransform(corners_right, H)

        corners_left = np.array([[0, 0], [w_left, 0], [w_left, h_left], [0, h_left]], dtype=np.float32).reshape(-1, 1, 2)
        all_corners = np.concatenate((corners_left, corners_right_trans), axis=0)

        [xmin, ymin] = np.int32(all_corners.min(axis=0).ravel() - 0.5)
        [xmax, ymax] = np.int32(all_corners.max(axis=0).ravel() + 0.5)
        translation = [-xmin, -ymin]

        size = (xmax - xmin, ymax - ymin)
        H_trans = np.array([[1, 0, translation[0]], [0, 1, translation[1]], [0, 0, 1]]) @ H

        warped_right = cv2.warpPerspective(img_right, H_trans, size)
        stitched = warped_right.copy()
        stitched[translation[1]:translation[1]+h_left, translation[0]:translation[0]+w_left] = img_left

        return stitched, translation, H_trans

    def process_images(self):
        """处理和发布图像"""
        if self.img_left is None or self.img_right is None:
            return

        try:
            stitched, translation, H_trans = self.warp_and_stitch(self.img_left, self.img_right, self.H)

            h_left, w_left = self.img_left.shape[:2]
            h_right, w_right = self.img_right.shape[:2]

            # 创建调试图像
            debug = stitched.copy()
            cv2.rectangle(debug,
                          (translation[0], translation[1]),
                          (translation[0] + w_left, translation[1] + h_left),
                          (0, 255, 0), 3)

            corners_right = np.array([[0, 0], [w_right, 0], [w_right, h_right], [0, h_right]], dtype=np.float32).reshape(-1, 1, 2)
            corners_right_trans = cv2.perspectiveTransform(corners_right, H_trans).astype(np.int32)
            cv2.polylines(debug, [corners_right_trans], isClosed=True, color=(0, 0, 255), thickness=3)

            # 发布图像
            self.pub_stitched.publish(self.bridge.cv2_to_imgmsg(stitched, "bgr8"))
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))

        except Exception as e:
            self.get_logger().error(f"图像处理错误: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DualCameraStitcher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点启动失败: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
