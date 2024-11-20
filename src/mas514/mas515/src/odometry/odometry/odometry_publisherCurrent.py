import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
import math


class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Initialize parameters for robot geometry
        self.wheel_radius = 0.0335  # meters
        self.wheel_base = 0.195     # meters
        self.prev_time = self.get_clock().now()

        # Initialize robot state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_wheel_rotation = 0.0
        self.right_wheel_rotation = 0.0

        # Initialize wheel velocities
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        # Set up ROS publishers
        self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Set up ROS Transform Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Set up ROS subscribers
        self.left_wheel_vel_sub = self.create_subscription(Float32, 'left_wheel_velocity', self.left_wheel_callback, 10)
        self.right_wheel_vel_sub = self.create_subscription(Float32, 'right_wheel_velocity', self.right_wheel_callback, 10)

        # Timer to publish odometry at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_odometry)

    def left_wheel_callback(self, msg):
        """Callback to update left wheel velocity."""
        self.left_wheel_velocity = msg.data

    def right_wheel_callback(self, msg):
        """Callback to update right wheel velocity."""
        self.right_wheel_velocity = msg.data

    def publish_odometry(self):
        """Compute and publish odometry based on wheel velocities."""
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        # Calculate linear and angular velocities of the robot
        v_left = self.left_wheel_velocity
        v_right = self.right_wheel_velocity
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheel_base

        # Update robot position and orientation
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = omega * dt
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Create and publish the Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

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
        self.left_wheel_rotation += self.left_wheel_velocity * dt
        self.right_wheel_rotation += self.right_wheel_velocity * dt

        # Publish joint states for the wheels
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['chassis_left_Wheel_joint', 'chassis_right_Wheel_joint']
        joint_state.position = [self.left_wheel_rotation, self.right_wheel_rotation]
        self.joint_state_pub.publish(joint_state)


# class InitialMapOdomPublisher(Node):
#     def __init__(self):
#         super().__init__('initial_map_odom_publisher')
#         self.tf_broadcaster = TransformBroadcaster(self)
#         self.publish_initial_transform()

#     def publish_initial_transform(self):
#         # Publish a one-time identity transform from map to odom
#         transform = TransformStamped()
#         transform.header.stamp = self.get_clock().now().to_msg()
#         transform.header.frame_id = 'map'
#         transform.child_frame_id = 'odom'

#         # Zero translation (identity transform)
#         transform.transform.translation.x = 0.0
#         transform.transform.translation.y = 0.0
#         transform.transform.translation.z = 0.0

#         # Zero rotation (identity quaternion)
#         q = quaternion_from_euler(0, 0, 0)
#         transform.transform.rotation.x = q[0]
#         transform.transform.rotation.y = q[1]
#         transform.transform.rotation.z = q[2]
#         transform.transform.rotation.w = q[3]

#         # Publish the initial identity transform
#         self.tf_broadcaster.sendTransform(transform)


# def main(args=None):
#     rclpy.init(args=args)
#     odometry_publisher = OdometryPublisher()
#     #initial_map_odom_publisher = InitialMapOdomPublisher()
#     try:
#         rclpy.spin(odometry_publisher)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         odometry_publisher.destroy_node()
#         #initial_map_odom_publisher.destroy_node()
#         rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    #odometry_publisher2 = InitialMapOdomPublisher()
    rclpy.spin(odometry_publisher)
    #rclpy.spin(odometry_publisher2)
    odometry_publisher.destroy_node()
    #odometry_publisher2.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
