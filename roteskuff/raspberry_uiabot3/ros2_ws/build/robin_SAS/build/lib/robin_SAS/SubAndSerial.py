import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
from time import sleep

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # Initialize subscription to /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Set up serial communication
        try:
            self.ser = serial.Serial(
                port='/dev/ttyUSB0',  # Adjust as per your device
                baudrate=115200,
                timeout=0.1
            )
            self.get_logger().info("Serial port opened successfully")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            self.ser = None

    def listener_callback(self, msg):
        # Extract linear and angular velocity from Twist message
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z*10
        # Log the received message to ensure callback is working
        self.get_logger().info(f"Received linear_vel: {linear_vel}, angular_vel: {angular_vel}")

        # Pack and send the values over serial
        if self.ser:
            try:
                # Pack 2 floats into a byte buffer ('=BBff' = 2 bytes for headers, 2 floats for velocities)
                buff = struct.pack('=BBff', 36, 36, angular_vel, linear_vel)
                self.ser.write(buff)
		#print(linear_vel)
                self.get_logger().info("Data sent to serial")

                sleep(0.1)

            except serial.SerialException as e:
                self.get_logger().error(f"Serial communication error: {e}")

def main(args=None):
    rclpy.init(args=args)

    serial_node = SerialNode()

    try:
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        serial_node.get_logger().info("Shutting down by user request...")

    # Clean up when done
    if serial_node.ser:
        serial_node.ser.close()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
