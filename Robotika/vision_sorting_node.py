#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


class UnifiedVisionSortingNode(Node):

    def __init__(self):
        super().__init__('unified_vision_sorting_node')
        self.bridge = CvBridge()

        # TF2 Inicijalizacija
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Kalibracioni parametri kamere
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.info_primljen = False
        
        # Visina kamere
        self.Z_c = 0.58  

        # Pretplate (Subscriptions)
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Publikacije (Publishers) za KOCKE
        self.red_world_pub = self.create_publisher(PointStamped, '/red_position', 10)
        self.green_world_pub = self.create_publisher(PointStamped, '/green_position', 10)
        self.blue_world_pub = self.create_publisher(PointStamped, '/blue_position', 10)

        # Publikacije (Publishers) za POVRŠI (Podloge)
        self.red_surface_pub = self.create_publisher(PointStamped, '/red_surface', 10)
        self.green_surface_pub = self.create_publisher(PointStamped, '/green_surface', 10)
        self.blue_surface_pub = self.create_publisher(PointStamped, '/blue_surface', 10)

        # Vizuelna slika
        self.image_pub = self.create_publisher(Image, '/filtered_image', 10)

        self.get_logger().info("Unified Vision Sorting Node (Kocke + Površi) je uspješno pokrenut.")

    def camera_info_callback(self, msg):
        if not self.info_primljen:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]
            if self.fx != 0.0 and self.fy != 0.0:
                self.info_primljen = True

    def find_all_objects(self, mask):
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        objects = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 100:  # Prag za eliminisanje šuma
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    objects.append({"center": (cx, cy), "area": area})
        
        objects = sorted(objects, key=lambda k: k['area'], reverse=True)
        return objects

    def process_and_publish(self, center, publisher, label_name, header):
        if not self.info_primljen or center is None:
            return None

        u = center[0]
        v = center[1]

        X_c = ((u - self.cx) * self.Z_c) / self.fx
        Y_c = ((v - self.cy) * self.Z_c) / self.fy

        point_cam = PointStamped()
        point_cam.header = header
        point_cam.header.frame_id = 'camera_color_optical_frame'
        point_cam.point.x = float(X_c)
        point_cam.point.y = float(Y_c)
        point_cam.point.z = float(self.Z_c)

        try:
            world_point = self.tf_buffer.transform(
                point_cam, 
                'link_base', 
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
            publisher.publish(world_point)
            self.get_logger().info(
                f"{label_name} (TF2) -> X={world_point.point.x:.3f} Y={world_point.point.y:.3f} Z={world_point.point.z:.3f}"
            )
            return world_point
        except Exception as e:
            self.get_logger().warning(f"TF2 greška za {label_name}: {str(e)}")
            return None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Maske za boje
        lower_red1, upper_red1 = np.array([0, 150, 50]), np.array([10, 255, 255])
        lower_red2, upper_red2 = np.array([170, 150, 50]), np.array([180, 255, 255])
        red_mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

        green_mask = cv2.inRange(hsv, np.array([40, 50, 50]), np.array([80, 255, 255]))
        blue_mask = cv2.inRange(hsv, np.array([100, 150, 50]), np.array([130, 255, 255]))

        output = cv_image.copy()

        # Pomoćna funkcija za obradu boja i slanje na odgovarajuće teme
        def process_color_objects(objects, color_name, cube_pub, surf_pub, bgr_color):
            for i, obj in enumerate(objects):
                center = obj["center"]
                # Prag od 8000 deli podloge od kocki (prilagodi po potrebi)
                is_surface = obj["area"] > 8000
                
                if is_surface:
                    label = f"{color_name}_Podloga"
                    self.process_and_publish(center, surf_pub, label, msg.header)
                else:
                    label = f"{color_name}_Kocka_{i+1}"
                    self.process_and_publish(center, cube_pub, label, msg.header)

                cv2.circle(output, center, 6, bgr_color, -1)
                cv2.putText(output, label, (center[0] + 10, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, bgr_color, 2)

        # Obrada sve tri boje
        process_color_objects(self.find_all_objects(red_mask), "RED", self.red_world_pub, self.red_surface_pub, (0, 0, 255))
        process_color_objects(self.find_all_objects(green_mask), "GREEN", self.green_world_pub, self.green_surface_pub, (0, 255, 0))
        process_color_objects(self.find_all_objects(blue_mask), "BLUE", self.blue_world_pub, self.blue_surface_pub, (255, 0, 0))

        try:
            ros_img = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
            ros_img.header = msg.header
            self.image_pub.publish(ros_img)
        except CvBridgeError as e:
            self.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedVisionSortingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
