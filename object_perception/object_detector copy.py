import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import tf2_ros

class ObjectPerception(Node):
    def __init__(self):
        super().__init__('object_perception')
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        
        self.color_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_object_pose', 10)
        self.depth_image = None

    def image_callback(self, color_msg):
        color_image = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        results = self.model(color_image)

        for result in results:
            if result.boxes:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    conf = box.conf[0].item()
                    cls = int(box.cls[0].item())

                    x_center = (x1 + x2) // 2
                    y_center = (y1 + y2) // 2

                    if self.depth_image is not None:
                        depth_region = self.depth_image[y1:y2, x1:x2]
                        z = np.mean(depth_region[depth_region > 0]) * 0.001

                        if z > 0:
                            pose_msg = PoseStamped()
                            pose_msg.header.frame_id = 'camera_link'
                            pose_msg.pose.position.x = x_center * z / 525.0
                            pose_msg.pose.position.y = y_center * z / 525.0
                            pose_msg.pose.position.z = z
                            pose_msg.pose.orientation.w = 1.0  # Orientação fixa para início
                            self.pose_pub.publish(pose_msg)
                            self.get_logger().info(f'Objeto detectado: X={pose_msg.pose.position.x:.2f}, Y={pose_msg.pose.position.y:.2f}, Z={pose_msg.pose.position.z:.2f}')

    def depth_callback(self, depth_msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectPerception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
