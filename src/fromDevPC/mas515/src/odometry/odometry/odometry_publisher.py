import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
import math


class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Robot parameters
        self.wheel_radius = 0.0335  # meters
        self.wheel_base = 0.195     # meters
        self.prev_time = self.get_clock().now()

        # Robot state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.delta_theta = 0.0
        self.left_wheel_rotation = 0.0
        self.right_wheel_rotation = 0.0
        

        # Wheel velocities
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0
        self.left_wheel_velocity_i = 0.0
        self.right_wheel_velocity_i = 0.0

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        self.left_pub = self.create_publisher(Float32, 'left_wheel_velocity', 10)
        self.right_pub = self.create_publisher(Float32, 'right_wheel_velocity', 10)

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to wheel speeds from /wheel_speed topic
        self.create_subscription(Float32MultiArray, '/wheel_speed', self.wheel_speed_callback, 10)

        # Timer to publish odometry at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_odometry)
        
        self.timer = self.create_timer(0.1, self.publish_velocity_l)
        self.timer = self.create_timer(0.1, self.publish_velocity_r)

    def wheel_speed_callback(self, msg):
        """Callback to update wheel velocities from /wheel_speed topic."""
        if len(msg.data) >= 2:
            self.left_wheel_velocity_i = msg.data[0]
            self.right_wheel_velocity_i = msg.data[1]

    def publish_odometry(self):
        """Compute and publish odometry based on wheel velocities."""
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        # Calculate linear and angular velocities of the robot
        v_left = -self.left_wheel_velocity_i #/ self.wheel_radius
        v_right = -self.right_wheel_velocity_i #/ self.wheel_radius
        v = (v_left + v_right) / 2.0
        omega = -(v_right - v_left) / self.wheel_base
        
        # Update robot position and orientation
        delta_x = v * math.cos(self.theta + self.delta_theta/2) * dt
        delta_y = v * math.sin(self.theta + self.delta_theta/2) * dt
        self.delta_theta = omega * dt
        self.x += delta_x
        self.y += delta_y
        self.theta += self.delta_theta

        # Create and publish the Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        #print("Wheel speed l", v_left)
        #print("Wheel speed r", v_right)

        # Set the orientation in the odometry message
        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = omega

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

        # Broadcast the transform from odom to base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # Update wheel joint rotations
        self.left_wheel_rotation += self.left_wheel_velocity_i * dt
        self.right_wheel_rotation += self.right_wheel_velocity_i * dt

        # Publish joint states for the wheels
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['chassis_left_Wheel_joint', 'chassis_right_Wheel_joint']
        joint_state.position = [self.left_wheel_rotation, self.right_wheel_rotation]
        self.joint_state_pub.publish(joint_state)
            
            
    def publish_velocity_l(self):
        left_wheel_velocity = Float32()
        left_wheel_velocity.data = 10*self.left_wheel_velocity_i/self.wheel_radius
        self.left_pub.publish(left_wheel_velocity)
        
        
    def publish_velocity_r(self):
        right_wheel_velocity = Float32()
        right_wheel_velocity.data = 10*self.right_wheel_velocity/self.wheel_radius
        self.right_pub.publish(right_wheel_velocity)   


def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
