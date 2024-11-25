import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        
        
        # Subscribe to wheel speeds from /wheel_speed topic
        self.create_subscription(Float32MultiArray, '/wheel_speed', self.wheel_speed_callback, 10)
        
        self.left_pub = self.create_publisher(Float32, 'left_wheel_velocity', 10)
        self.right_pub = self.create_publisher(Float32, 'right_wheel_velocity', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)  # Publish every 0.1 seconds
        
    def wheel_speed_callback(self, msg):
        """Callback to update wheel velocities from /wheel_speed topic."""
        if len(msg.data) >= 2:
            self.left_wheel_velocity = msg.data[0]
            self.right_wheel_velocity = msg.data[1]
            
            
    def publish_velocity(self):
        velocity_msg = Float32()
        self.velocity_l_msg.data = self.left_wheel_velocity
        self.velocity_r_msg.data = self.right_wheel_velocity
        self.left_pub.publish(self.elocity_l_msg)
        self.right_pub.publish(self.velocity_r_msg)
        

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
