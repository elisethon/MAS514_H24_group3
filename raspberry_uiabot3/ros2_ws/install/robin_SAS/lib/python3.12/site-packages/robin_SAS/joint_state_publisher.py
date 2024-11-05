import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
import numpy as np

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        # Distance between wheels
        self.b = 0.195  # Distance between wheels (in meters)
        
        # Initial positions
        self.x = 0.0  # x position
        self.y = 0.0  # y position
        self.theta = 0.0  # Orientation (in radians)
        
        # Subscriber to /wheel_speed
        self.subscription = self.create_subscription(
            Float32MultiArray, 
            '/wheel_speed', 
            self.wheel_speed_callback, 
            10
        )

        # Publisher for /joint_states
        self.joint_state_publisher = self.create_publisher(
            JointState, 
            '/joint_states', 
            10
        )

        # Publisher for /joint_state_odom
        self.odom_publisher = self.create_publisher(
            Pose, 
            '/joint_state_odom', 
            10
        )

        # Assuming the robot has left and right wheel joints
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0

    def wheel_speed_callback(self, msg):
        # Extract wheel speeds (left and right wheel)
        left_wheel_speed, right_wheel_speed = msg.data

        # Estimate the movement of the wheels
        delta_s_r = right_wheel_speed * 0.1  # Change in distance for right wheel (0.1 sec timestep)
        delta_s_l = left_wheel_speed * 0.1   # Change in distance for left wheel (0.1 sec timestep)
        
        # Calculate delta_s and delta_theta for odometry
        delta_s = (delta_s_r + delta_s_l) / 2.0
        delta_theta = (delta_s_r - delta_s_l) / self.b

        # Update the pose based on the equations
        self.x += delta_s * np.cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * np.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # Update wheel positions (for the joint state message)
        self.left_wheel_position += delta_s_l
        self.right_wheel_position += delta_s_r

        # Create a JointState message for wheel positions
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state_msg.position = [self.left_wheel_position, self.right_wheel_position]
        joint_state_msg.velocity = [left_wheel_speed, right_wheel_speed]

        # Publish the joint state
        self.joint_state_publisher.publish(joint_state_msg)

        # Create a Pose message for the odometry
        pose_msg = Pose()
        pose_msg.position.x = self.x
        pose_msg.position.y = self.y
        pose_msg.position.z = self.theta  # Use z to store the orientation theta

        # Publish the odometry pose
        self.odom_publisher.publish(pose_msg)

    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()

    try:
        joint_state_publisher.get_logger().info("JointStatePublisher with odometry calculation is now running...")
        joint_state_publisher.run()
    except KeyboardInterrupt:
        joint_state_publisher.get_logger().info("Shutdown requested by user...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
