import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')
        self.broadcaster = StaticTransformBroadcaster(self)

        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'tool_frame'
        static_transform.child_frame_id = 'camera_link'

        # Configure a posição e orientação da câmera em relação à ferramenta
        static_transform.transform.translation.x = 0.1
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.05
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(static_transform)

def main(args=None):
    rclpy.init(args=args)
    node = StaticTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
