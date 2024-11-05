import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.left_pub = self.create_publisher(Float32, 'left_wheel_velocity', 10)
        self.right_pub = self.create_publisher(Float32, 'right_wheel_velocity', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)  # Publish every 0.1 seconds

    def publish_velocity(self):
        velocity_msg = Float32()
        velocity_msg.data = 2.0  # 2.0 rad/s for each wheel to get 0.2 m/s robot speed
        self.left_pub.publish(velocity_msg)
        self.right_pub.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
