#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import queue

class ImageStitchNode(Node):
    def __init__(self):
        super().__init__('image_stitch_node')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # QoS configuration to match publisher
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Declare parameters with default values
        self.declare_parameter('input_topic', '/stitched_image')
        self.declare_parameter('output_topic', '/cropped_image')
        self.declare_parameter('debug_topic', '/debug_image')
        
        # ROI parameters
        self.declare_parameter('roi_x_offset_ratio', 0.1)
        self.declare_parameter('roi_y_offset_ratio', 0.1)
        self.declare_parameter('roi_width_ratio', 0.7)
        self.declare_parameter('roi_height_ratio', 0.8)
        
        # Stitching parameters
        self.declare_parameter('min_shift', 1)
        self.declare_parameter('max_shift', 200)
        self.declare_parameter('max_width', 10000000)
        self.declare_parameter('auto_reset', False)
        self.declare_parameter('reset_now', False)
        self.declare_parameter('stitch_along_y', True)
        
        # Feature detection parameters
        self.declare_parameter('orb_features', 1000)
        self.declare_parameter('match_ratio', 0.75)
        self.declare_parameter('min_matches', 4)
        
        # Get initial parameter values
        input_topic_param = self.get_parameter('input_topic').value
        output_topic_param = self.get_parameter('output_topic').value
        debug_topic_param = self.get_parameter('debug_topic').value
        
        self.input_topic = str(input_topic_param) if input_topic_param is not None else '/stitched_image'
        self.output_topic = str(output_topic_param) if output_topic_param is not None else '/cropped_image'
        self.debug_topic = str(debug_topic_param) if debug_topic_param is not None else '/debug_image'
        
        roi_x_param = self.get_parameter('roi_x_offset_ratio').value
        roi_y_param = self.get_parameter('roi_y_offset_ratio').value
        roi_w_param = self.get_parameter('roi_width_ratio').value
        roi_h_param = self.get_parameter('roi_height_ratio').value
        
        self.roi_x_offset_ratio = float(roi_x_param) if roi_x_param is not None else 0.1
        self.roi_y_offset_ratio = float(roi_y_param) if roi_y_param is not None else 0.1
        self.roi_width_ratio = float(roi_w_param) if roi_w_param is not None else 0.7
        self.roi_height_ratio = float(roi_h_param) if roi_h_param is not None else 0.8
        
        min_shift_param = self.get_parameter('min_shift').value
        max_shift_param = self.get_parameter('max_shift').value
        max_width_param = self.get_parameter('max_width').value
        auto_reset_param = self.get_parameter('auto_reset').value
        reset_now_param = self.get_parameter('reset_now').value
        stitch_along_y_param = self.get_parameter('stitch_along_y').value
        
        self.min_shift = int(min_shift_param) if min_shift_param is not None else 1
        self.max_shift = int(max_shift_param) if max_shift_param is not None else 200
        self.max_width = int(max_width_param) if max_width_param is not None else 10000000
        self.auto_reset = bool(auto_reset_param) if auto_reset_param is not None else False
        self.reset_now = bool(reset_now_param) if reset_now_param is not None else False
        self.stitch_along_y = bool(stitch_along_y_param) if stitch_along_y_param is not None else True
        
        orb_features_param = self.get_parameter('orb_features').value
        match_ratio_param = self.get_parameter('match_ratio').value
        min_matches_param = self.get_parameter('min_matches').value
        
        self.orb_features = int(orb_features_param) if orb_features_param is not None else 1000
        self.match_ratio = float(match_ratio_param) if match_ratio_param is not None else 0.75
        self.min_matches = int(min_matches_param) if min_matches_param is not None else 4
        
        # Initialize ORB detector
        self.orb_detector = cv2.ORB_create(self.orb_features)
        
        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Image processing variables
        self.panorama = None
        self.last_roi = None
        self.panorama_lock = threading.Lock()
        
        # Create publishers
        self.stitched_pub = self.create_publisher(Image, self.output_topic, self.qos_profile)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, self.qos_profile)
        self.debug_roi_pub = self.create_publisher(Image, f"{self.debug_topic}/roi", self.qos_profile)
        self.debug_matches_pub = self.create_publisher(Image, f"{self.debug_topic}/matches", self.qos_profile)
        
        # Create subscriber with matching QoS
        self.image_sub = self.create_subscription(
            Image, self.input_topic, self.image_callback, self.qos_profile)
        
        self.get_logger().info(f"ImageStitchNode initialized")
        self.get_logger().info(f"Subscribing to: {self.input_topic}")
        self.get_logger().info(f"Publishing to: {self.output_topic}")
    
    def parameters_callback(self, params):
        """Handle parameter changes"""
        result = SetParametersResult()
        result.successful = True
        
        for param in params:
            if param.name == 'roi_x_offset_ratio':
                self.roi_x_offset_ratio = param.value
            elif param.name == 'roi_y_offset_ratio':
                self.roi_y_offset_ratio = param.value
            elif param.name == 'roi_width_ratio':
                self.roi_width_ratio = param.value
            elif param.name == 'roi_height_ratio':
                self.roi_height_ratio = param.value
            elif param.name == 'min_shift':
                self.min_shift = param.value
            elif param.name == 'max_shift':
                self.max_shift = param.value
            elif param.name == 'max_width':
                self.max_width = param.value
            elif param.name == 'auto_reset':
                self.auto_reset = param.value
            elif param.name == 'reset_now':
                self.reset_now = param.value
                if self.reset_now:
                    self.reset_panorama()
                    self.get_logger().info("Panorama reset")
            elif param.name == 'stitch_along_y':
                self.stitch_along_y = param.value
            elif param.name == 'orb_features':
                self.orb_features = param.value
                self.orb_detector = cv2.ORB_create(self.orb_features)
            elif param.name == 'match_ratio':
                self.match_ratio = param.value
            elif param.name == 'min_matches':
                self.min_matches = param.value
        
        return result
    
    def image_callback(self, msg):
        """Process incoming image messages"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge exception: {e}")
            return
        
        if cv_image is None or cv_image.size == 0:
            self.get_logger().warn("Received empty image")
            return
        
        self.process_image(cv_image, msg.header)
    
    def process_image(self, img, header):
        """Main image processing pipeline"""
        # Extract ROI
        roi, roi_rect = self.extract_roi(img)
        
        # Publish debug image with ROI highlighted
        self.publish_debug_image(img, roi_rect, header)
        
        # Stitch images
        self.stitch_images(roi, header)
    
    def extract_roi(self, img):
        """Extract Region of Interest from image"""
        H, W = img.shape[:2]
        
        # Calculate ROI dimensions
        x_offset = int(W * self.roi_x_offset_ratio)
        y_offset = int(H * self.roi_y_offset_ratio)
        width = int(W * self.roi_width_ratio)
        height = int(H * self.roi_height_ratio)
        
        # Ensure ROI is within image bounds
        x_offset = max(0, min(x_offset, W - 1))
        y_offset = max(0, min(y_offset, H - 1))
        width = max(1, min(width, W - x_offset))
        height = max(1, min(height, H - y_offset))
        
        roi_rect = (x_offset, y_offset, width, height)
        roi = img[y_offset:y_offset+height, x_offset:x_offset+width].copy()
        
        return roi, roi_rect
    
    def publish_debug_image(self, img, roi_rect, header):
        """Publish debug image with ROI highlighted"""
        debug_img = img.copy()
        
        # Draw ROI rectangle in green
        x, y, w, h = roi_rect
        cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Add text annotation
        text = f"ROI: {x},{y} {w}x{h}"
        cv2.putText(debug_img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
        debug_msg.header = header
        self.debug_pub.publish(debug_msg)
    
    def stitch_images(self, current_roi, header):
        """Stitch current ROI with panorama"""
        with self.panorama_lock:
            # Initialize panorama if empty or reset requested
            if self.auto_reset or self.panorama is None or self.last_roi is None:
                self.get_logger().info("Initializing panorama with first frame")
                self.panorama = current_roi.copy()
                self.last_roi = current_roi.copy()
                self.publish_stitched_image(header)
                return
            
            # Resize if size mismatch
            roi_to_process = current_roi
            if current_roi.shape != self.last_roi.shape:
                self.get_logger().warn("Size mismatch, resizing current ROI")
                roi_to_process = cv2.resize(current_roi, (self.last_roi.shape[1], self.last_roi.shape[0]))
            
            # Detect and match features
            kp_prev, desc_prev = self.detect_features(self.last_roi)
            kp_cur, desc_cur = self.detect_features(roi_to_process)
            
            if desc_prev is None or desc_cur is None:
                self.get_logger().warn("Feature detection failed")
                self.last_roi = roi_to_process.copy()
                return
            
            good_matches = self.match_features(desc_prev, desc_cur)
            if len(good_matches) < self.min_matches:
                self.get_logger().warn("Feature matching failed")
                self.last_roi = roi_to_process.copy()
                return
            
            # Estimate transform
            offset = self.estimate_transform(kp_prev, kp_cur, good_matches)
            if offset is None:
                self.get_logger().warn("Transform estimation failed")
                self.last_roi = roi_to_process.copy()
                return
            
            # Publish match visualization
            self.publish_match_visualization(self.last_roi, roi_to_process, kp_prev, kp_cur, good_matches, header)
            
            # Check shift validity
            shift = int(round(offset))
            if abs(shift) < self.min_shift or abs(shift) > self.max_shift:
                self.get_logger().warn(f"Shift {shift} out of valid range [{self.min_shift}, {self.max_shift}]")
                self.last_roi = roi_to_process.copy()
                return
            
            # Perform stitching
            self.perform_stitching(roi_to_process, shift)
            
            # Update last ROI
            self.last_roi = roi_to_process.copy()
            
            # Publish stitched result
            self.publish_stitched_image(header)
    
    def detect_features(self, img):
        """Detect ORB features in image"""
        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Apply histogram equalization
        gray = cv2.equalizeHist(gray)
        
        # Detect and compute features
        kp, desc = self.orb_detector.detectAndCompute(gray, None)
        
        return kp, desc
    
    def match_features(self, desc1, desc2):
        """Match features between two descriptors"""
        if desc1 is None or desc2 is None:
            return []
        
        # BF matcher with KNN
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        knn_matches = matcher.knnMatch(desc1, desc2, k=2)
        
        # Apply Lowe's ratio test
        good_matches = []
        for match_pair in knn_matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < self.match_ratio * n.distance:
                    good_matches.append(m)
        
        return good_matches
    
    def estimate_transform(self, kp1, kp2, matches):
        """Estimate transform between matched keypoints"""
        if len(matches) < self.min_matches:
            return None
        
        # Extract matched points
        pts1_list = [kp1[m.queryIdx].pt for m in matches]
        pts2_list = [kp2[m.trainIdx].pt for m in matches]
        pts1 = np.float32(pts1_list).reshape(-1, 1, 2)
        pts2 = np.float32(pts2_list).reshape(-1, 1, 2)
        
        # Estimate affine transform using RANSAC
        try:
            H, mask = cv2.estimateAffinePartial2D(pts1, pts2, method=cv2.RANSAC, ransacReprojThreshold=3.0)
            if H is None:
                return None
            
            # Extract offset
            offset = H[1, 2] if self.stitch_along_y else H[0, 2]
            
            return offset
        except Exception as e:
            self.get_logger().error(f"Transform estimation error: {e}")
            return None
    
    def publish_match_visualization(self, img1, img2, kp1, kp2, matches, header):
        """Publish visualization of feature matches"""
        # Limit number of matches for visualization
        matches_to_show = matches[:min(50, len(matches))]
        
        match_img = cv2.drawMatches(img1, kp1, img2, kp2, matches_to_show, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        
        match_msg = self.bridge.cv2_to_imgmsg(match_img, "bgr8")
        match_msg.header = header
        self.debug_matches_pub.publish(match_msg)
    
    def perform_stitching(self, current_roi, shift):
        """Perform the actual stitching operation"""
        if self.panorama is None:
            return
            
        add_len = abs(shift)
        if add_len == 0:
            return
        
        if not self.stitch_along_y:
            # Horizontal stitching
            if shift > 0:
                strip = current_roi[:, :add_len].copy()
                self.panorama = np.hstack([strip, self.panorama])
            else:
                strip = current_roi[:, -add_len:].copy()
                self.panorama = np.hstack([self.panorama, strip])
            
            # Crop if too wide
            if self.panorama.shape[1] > self.max_width:
                off = self.panorama.shape[1] - self.max_width
                self.panorama = self.panorama[:, off:].copy()
        else:
            # Vertical stitching
            if self.panorama.shape[1] != current_roi.shape[1]:
                self.get_logger().warn("Column mismatch in vertical stitching, resetting")
                self.panorama = current_roi.copy()
                return
            
            if shift > 0:
                strip = current_roi[:add_len, :].copy()
                self.panorama = np.vstack([strip, self.panorama])
            else:
                strip = current_roi[-add_len:, :].copy()
                self.panorama = np.vstack([self.panorama, strip])
        
        self.get_logger().debug(f"Stitched with shift: {shift}, panorama size: {self.panorama.shape}")
    
    def publish_stitched_image(self, header):
        """Publish the stitched panorama image"""
        if self.panorama is None:
            return
        
        stitched_msg = self.bridge.cv2_to_imgmsg(self.panorama, "bgr8")
        stitched_msg.header = header
        self.stitched_pub.publish(stitched_msg)
    
    def reset_panorama(self):
        """Reset the panorama and processing state"""
        with self.panorama_lock:
            self.panorama = None
            self.last_roi = None

def main(args=None):
    rclpy.init(args=args)
    node = ImageStitchNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 