#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile
from math import sin, cos
import tf_transformations

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Set parameters for wheel separation and radius
        self.wheel_separation = 0.4  # meters (adjust as needed)
        self.wheel_radius = 0.1  # meters (adjust as needed)

        # Robot position in odom frame
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Robot velocity in odom frame
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        # Timestamps and odometry calculations
        self.last_time = self.get_clock().now()

        # Subscriber and publisher setup
        qos_profile = QoSProfile(depth=10)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile
        )
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos_profile)

    def odom_callback(self, msg):
        # Calculate odometry based on wheel encoders or other sensors
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Using the received odometry velocities
        delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
        delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
        delta_th = self.vth * dt

        # Update the robot's estimated position
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Create quaternion from yaw
        odom_quat = Quaternion()
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.th)
        odom_quat.x = quaternion[0]
        odom_quat.y = quaternion[1]
        odom_quat.z = quaternion[2]
        odom_quat.w = quaternion[3]

        # Publish the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        # Set the position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = odom_quat

        # Set the velocity
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.vth

        # Publish the odometry
        self.odom_pub.publish(odom_msg)

        # Broadcast the transforms
        self.broadcast_transforms(current_time, odom_msg.pose.pose)

        # Update time for next calculation
        self.last_time = current_time

    def broadcast_transforms(self, current_time, pose):
        # Map to Odom (SLAM or localization override)
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = current_time.to_msg()
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "odom"
        map_to_odom.transform.translation.x = 0.0  # Set by SLAM or localization
        map_to_odom.transform.translation.y = 0.0  # Set by SLAM or localization
        map_to_odom.transform.translation.z = 0.0
        map_to_odom.transform.rotation.x = 0.0
        map_to_odom.transform.rotation.y = 0.0
        map_to_odom.transform.rotation.z = 0.0
        map_to_odom.transform.rotation.w = 1.0  # Placeholder; SLAM will override
        self.tf_broadcaster.sendTransform(map_to_odom)

        # Odom to Base Footprint
        odom_to_base_footprint = TransformStamped()
        odom_to_base_footprint.header.stamp = current_time.to_msg()
        odom_to_base_footprint.header.frame_id = "odom"
        odom_to_base_footprint.child_frame_id = "base_footprint"
        odom_to_base_footprint.transform.translation.x = pose.position.x
        odom_to_base_footprint.transform.translation.y = pose.position.y
        odom_to_base_footprint.transform.translation.z = 0.0
        odom_to_base_footprint.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(odom_to_base_footprint)

        # Base Footprint to Base Link (Fixed Transform)
        base_footprint_to_base_link = TransformStamped()
        base_footprint_to_base_link.header.stamp = current_time.to_msg()
        base_footprint_to_base_link.header.frame_id = "base_footprint"
        base_footprint_to_base_link.child_frame_id = "base_link"
        base_footprint_to_base_link.transform.translation.x = 0.0
        base_footprint_to_base_link.transform.translation.y = 0.0
        base_footprint_to_base_link.transform.translation.z = 0.0
        base_footprint_to_base_link.transform.rotation.x = 0.0
        base_footprint_to_base_link.transform.rotation.y = 0.0
        base_footprint_to_base_link.transform.rotation.z = 0.0
        base_footprint_to_base_link.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(base_footprint_to_base_link)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
