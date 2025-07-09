#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import cv2
import numpy as np
import os
import sys
import select
import termios
import tty
import yaml
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import threading

class DualCameraCalibration(Node):
    def __init__(self):
        super().__init__('dual_camera_calibration')
        
        # 全局设置
        self.bridge = CvBridge()
        self.PATTERN_SIZE = (6, 5)
        # Flags 组合：
        #   CALIB_CB_ADAPTIVE_THRESH  - 自适应阈值处理，增强局部对比
        #   CALIB_CB_NORMALIZE_IMAGE  - 归一化灰度，减小光照不均影响
        #   CALIB_CB_FAST_CHECK       - 先做快速二值检测，提高速度
        self.FLAGS = (
                cv2.CALIB_CB_ADAPTIVE_THRESH
                + cv2.CALIB_CB_NORMALIZE_IMAGE
                + cv2.CALIB_CB_FAST_CHECK
        )

        # 保存目录
        self.SAVE_DIR = "calibration_images"
        os.makedirs(self.SAVE_DIR, exist_ok=True)
        # 标定结果目录（npy和yaml）
        self.RESULT_DIR = "calibration_files"
        os.makedirs(self.RESULT_DIR, exist_ok=True)

        # QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # 发布器
        self.pub_left_corners = self.create_publisher(Image, '/left_corners', qos_profile)
        self.pub_right_corners = self.create_publisher(Image, '/right_corners', qos_profile)
        self.pub_debug = self.create_publisher(Image, '/debug_image', qos_profile)
        # 新增二值图像发布
        self.pub_left_binary = self.create_publisher(Image, '/left_binary', qos_profile)
        self.pub_right_binary = self.create_publisher(Image, '/right_binary', qos_profile)

        # 临时存储最新图像与角点
        self.img_left = None
        self.img_right = None
        self.corners_left = None
        self.corners_right = None
        self.ret_l = False
        self.ret_r = False

        # 多张变换矩阵列表及平均矩阵
        self.Hs = []
        self.H_avg = None
        self.active = True  # 标志节点是否活跃

        # Terminal 设置，用于捕获键盘输入
        self.old_term = termios.tcgetattr(sys.stdin)
        self.init_term()

        # 订阅器和同步器
        self.left_sub = Subscriber(self, Image, "/hk_camera/left_camera/image_raw", qos_profile=qos_profile)
        self.right_sub = Subscriber(self, Image, "/hk_camera/right_camera/image_raw", qos_profile=qos_profile)
        self.sync = ApproximateTimeSynchronizer([self.left_sub, self.right_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.callback)

        self.get_logger().info("[INFO] 双相机标定启动: 终端键入 'c' 保存并更新 H, 'q' 退出并生成 npy 和 yaml")

        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

    def init_term(self):
        tty.setcbreak(sys.stdin.fileno())

    def restore_term(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_term)

    def convert_npy_to_yaml(self, npy_path, yaml_path=None):
        """
        Convert .npy homography matrix file to YAML format
        
        Args:
            npy_path: Path to .npy file
            yaml_path: Output YAML path (optional, defaults to same name with .yaml extension)
        """
        if not os.path.exists(npy_path):
            self.get_logger().error(f"Error: {npy_path} does not exist")
            return False
        
        try:
            # Load numpy array
            H = np.load(npy_path)
            self.get_logger().info(f"Loaded homography matrix from {npy_path}")
            self.get_logger().info(f"Matrix shape: {H.shape}")
            self.get_logger().info(f"Matrix:\n{H}")
            
            # Validate matrix
            if H.shape != (3, 3):
                self.get_logger().error(f"Error: Expected 3x3 matrix, got {H.shape}")
                return False
            
            # Determine output path
            if yaml_path is None:
                yaml_path = npy_path.replace('.npy', '.yaml')
            
            # Write YAML file with OpenCV format
            with open(yaml_path, 'w') as f:
                f.write("%YAML:1.0\n")
                f.write("---\n")
                f.write("homography: !!opencv-matrix\n")
                f.write("   rows: 3\n")
                f.write("   cols: 3\n")
                f.write("   dt: d\n")
                f.write("   data: [ ")
                data_str = ", ".join([f"{x:.16e}" for x in H.flatten()])
                f.write(data_str)
                f.write(" ]\n")
            
            self.get_logger().info(f"Successfully converted to {yaml_path}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error during conversion: {e}")
            return False

    # 边缘模糊 + 简单二值化预处理：CLAHE -> 双边滤波 -> 全局阈值
    def preprocess(self, gray):
        # 自适应直方图均衡化
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        eq = clahe.apply(gray)
        # 双边滤波保留边缘
        blurred = cv2.bilateralFilter(eq, d=9, sigmaColor=75, sigmaSpace=75)
        # 简单二值化，阈值设为128
        _, binary = cv2.threshold(blurred, 40, 255, cv2.THRESH_BINARY)
        return binary

    # 角点可视化并发布
    def publish_corners(self, img, corners, ret, pub):
        vis = img.copy()
        if ret:
            cv2.drawChessboardCorners(vis, self.PATTERN_SIZE, corners, ret)
        pub.publish(self.bridge.cv2_to_imgmsg(vis, "bgr8"))

    # 图像拼接函数，返回 stitched 和 debug 图
    def warp_and_stitch(self, img_l, img_r, H):
        h_l, w_l = img_l.shape[:2]
        h_r, w_r = img_r.shape[:2]
        corners_r = np.array([[0,0],[w_r,0],[w_r,h_r],[0,h_r]], dtype=np.float32).reshape(-1,1,2)
        corners_r_tr = cv2.perspectiveTransform(corners_r, H)
        corners_l = np.array([[0,0],[w_l,0],[w_l,h_l],[0,h_l]], dtype=np.float32).reshape(-1,1,2)
        all_c = np.concatenate((corners_l, corners_r_tr), axis=0)
        xmin, ymin = np.int32(all_c.min(axis=0).ravel() - 0.5)
        xmax, ymax = np.int32(all_c.max(axis=0).ravel() + 0.5)
        tr = [-xmin, -ymin]
        size = (xmax - xmin, ymax - ymin)
        H_tr = np.array([[1,0,tr[0]],[0,1,tr[1]],[0,0,1]]) @ H

        warped_r = cv2.warpPerspective(img_r, H_tr, size)
        stitched = warped_r.copy()
        stitched[tr[1]:tr[1]+h_l, tr[0]:tr[0]+w_l] = img_l

        debug = stitched.copy()
        # 左图绿框
        cv2.rectangle(debug, (tr[0], tr[1]), (tr[0]+w_l, tr[1]+h_l), (0,255,0), 2)
        # 右图红框
        pts_r = np.int32(cv2.perspectiveTransform(corners_r, H_tr))
        cv2.polylines(debug, [pts_r], True, (0,0,255), 2)
        return stitched, debug

    # 同步回调：预处理后检测，发布角点、二值图和基于 H_avg 发布 debug 图
    def callback(self, img_left_msg, img_right_msg):
        if not self.active:
            return
        self.img_left = self.bridge.imgmsg_to_cv2(img_left_msg, "bgr8")
        self.img_right = self.bridge.imgmsg_to_cv2(img_right_msg, "bgr8")

        gray_l = cv2.cvtColor(self.img_left, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(self.img_right, cv2.COLOR_BGR2GRAY)
        # 使用二值化预处理
        proc_l = self.preprocess(gray_l)
        proc_r = self.preprocess(gray_r)

        # 发布二值化图像
        self.pub_left_binary.publish(self.bridge.cv2_to_imgmsg(proc_l, "mono8"))
        self.pub_right_binary.publish(self.bridge.cv2_to_imgmsg(proc_r, "mono8"))

        # 棋盘格角点检测
        self.ret_l, self.corners_left = cv2.findChessboardCorners(proc_l, self.PATTERN_SIZE, self.FLAGS)
        self.ret_r, self.corners_right = cv2.findChessboardCorners(proc_r, self.PATTERN_SIZE, self.FLAGS)

        self.publish_corners(self.img_left, self.corners_left, self.ret_l, self.pub_left_corners)
        self.publish_corners(self.img_right, self.corners_right, self.ret_r, self.pub_right_corners)

        if self.H_avg is not None:
            _, debug = self.warp_and_stitch(self.img_left, self.img_right, self.H_avg)
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))

    # 键盘处理：Terminal 读取 'c'/'q'
    def process_key(self):
        dr, dw, de = select.select([sys.stdin], [], [], 0)
        if dr:
            c = sys.stdin.read(1)
            if c == 'c' and self.ret_l and self.ret_r:
                idx = len(self.Hs)
                cv2.imwrite(os.path.join(self.SAVE_DIR, f"left_{idx}.png"), self.img_left)
                cv2.imwrite(os.path.join(self.SAVE_DIR, f"right_{idx}.png"), self.img_right)
                self.get_logger().info(f"[SAVE] 图像对 {idx} 保存到 {self.SAVE_DIR}")

                H, _ = cv2.findHomography(self.corners_right, self.corners_left)
                self.Hs.append(H)
                self.H_avg = np.mean(np.stack(self.Hs), axis=0)
                self.get_logger().info(f"[CALC] 新单应加入，共 {len(self.Hs)} 张，已更新平均 H")

            elif c == 'q':
                if self.H_avg is not None:
                    npy_path = os.path.join(self.RESULT_DIR, "H_right_to_left.npy")
                    yaml_path = os.path.join(self.RESULT_DIR, "H_right_to_left.yaml")
                    # 保存 .npy 文件
                    np.save(npy_path, self.H_avg)
                    self.get_logger().info(f"[DONE] 平均单应矩阵保存为 {npy_path}")
                    # 自动转换为 .yaml 文件
                    if self.convert_npy_to_yaml(npy_path, yaml_path):
                        self.get_logger().info(f"[CONVERT] 成功转换为 YAML 格式: {yaml_path}")
                    else:
                        self.get_logger().error("[CONVERT] YAML 转换失败")
                self.active = False  # 先设置为False，防止publish
                self.get_logger().info("用户退出，标定完成")
                rclpy.shutdown()

    def keyboard_listener(self):
        """键盘监听线程"""
        while rclpy.ok():
            try:
                self.process_key()
            except Exception as e:
                self.get_logger().error(f"键盘监听异常: {e}")
                break

    def destroy_node(self):
        """清理资源"""
        self.restore_term()
        super().destroy_node()

# 主入口
def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DualCameraCalibration()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        # 只在 rclpy 还没 shutdown 时调用
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()