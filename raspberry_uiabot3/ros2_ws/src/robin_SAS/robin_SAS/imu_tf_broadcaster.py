import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler

class IMUTfBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')

        # Initialize a broadcaster
        self.br = TransformBroadcaster(self)

        # Subscribe to the IMU data
        self.sub = self.create_subscription(
            Imu,
            '/bno055/imu',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg):
        # Broadcast the transform from base_link to imu_link
        t = TransformStamped()

        # Fill the transform data
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'

        # Use the IMU orientation from the message
        t.transform.rotation = msg.orientation

        # Broadcast the transform
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = IMUTfBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
