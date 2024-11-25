import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
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
        self.left_wheel_rotation = 0.0
        self.right_wheel_rotation = 0.0

        # Wheel velocities
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Transform broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Broadcast static transforms: map -> odom and base_footprint -> base_link
        self.broadcast_static_transforms()

        # Subscribe to wheel speeds from /wheel_speed topic
        self.create_subscription(Float32MultiArray, '/wheel_speed', self.wheel_speed_callback, 10)

        # Subscribe to filtered odometry from robot_localization
        self.create_subscription(Odometry, '/odometry/filtered', self.filtered_odometry_callback, 10)

        # Timer to publish odometry at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_odometry)

    def broadcast_static_transforms(self):
        """Broadcast static transforms for map -> odom and base_footprint -> base_link."""

        # Static transform for map -> odom
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = self.get_clock().now().to_msg()
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'odom'
        map_to_odom.transform.translation.x = 0.0
        map_to_odom.transform.translation.y = 0.0
        map_to_odom.transform.translation.z = 0.0
        map_to_odom.transform.rotation.w = 1.0  # Identity quaternion (no rotation)

        # Static transform for base_footprint -> base_link
        base_footprint_to_base_link = TransformStamped()
        base_footprint_to_base_link.header.stamp = self.get_clock().now().to_msg()
        base_footprint_to_base_link.header.frame_id = 'base_footprint'
        base_footprint_to_base_link.child_frame_id = 'base_link'
        base_footprint_to_base_link.transform.translation.x = 0.0
        base_footprint_to_base_link.transform.translation.y = 0.0
        base_footprint_to_base_link.transform.translation.z = 0.0
        base_footprint_to_base_link.transform.rotation.w = 1.0  # Identity quaternion (no rotation)

        # Send both static transforms
        self.static_tf_broadcaster.sendTransform([map_to_odom, base_footprint_to_base_link])

    def wheel_speed_callback(self, msg):
        """Callback to update wheel velocities from /wheel_speed topic."""
        if len(msg.data) >= 2:
            self.left_wheel_velocity = msg.data[0]
            self.right_wheel_velocity = msg.data[1]

    def filtered_odometry_callback(self, msg):
        """Callback to update map -> odom with filtered odometry data."""

        # Broadcast dynamic transform from map to odom using filtered odometry data
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'  # Set 'map' as parent of 'odom'
        t.child_frame_id = 'odom'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # Publish the filtered odometry message to odom topic
        self.odom_pub.publish(msg)
        
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

        # Create and publish the Odometry message (raw odometry)
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
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

        # Publish the raw odometry message
        self.odom_pub.publish(odom_msg)

        # Update wheel joint rotations
        self.left_wheel_rotation += self.left_wheel_velocity * dt
        self.right_wheel_rotation += self.right_wheel_velocity * dt

        # Publish joint states for the wheels
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['chassis_left_Wheel_joint', 'chassis_right_Wheel_joint']
        joint_state.position = [self.left_wheel_rotation, self.right_wheel_rotation]
        self.joint_state_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
