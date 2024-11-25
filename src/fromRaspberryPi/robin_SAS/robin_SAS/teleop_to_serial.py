import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
from time import sleep
from std_msgs.msg import Float32MultiArray

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # 1. Initialize subscriber to /cmd_vel topic
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)

        # 2. Initialize publisher for wheel speeds on /wheel_speed topic
        self.wheel_speed_publisher = self.create_publisher(Float32MultiArray, '/wheel_speed', 10)

        # 3. Set up serial communication
        self.initialize_serial()

        # 4. Send initial zeros to clear any leftover data on the ESP side
        self.send_initial_zeros()

        # 5. Set up a timer to continuously check for incoming serial data
        self.timer = self.create_timer(0.2, self.read_serial_data)

    def initialize_serial(self):
        """Initialize or reset the serial connection."""
        try:
            self.ser = serial.Serial(
                port='/dev/ttyUSB1',  # MIGHT NEED TO BE CHANGED: DEPENDS ON ORDER OF CONNECTION TO THE USB
                baudrate=115200,
                timeout=0.1  # Short timeout for non-blocking read
            )
            self.get_logger().info("Serial port opened successfully")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            self.ser = None

    def send_initial_zeros(self):
        """Send a packet of zeros to 'clean' the serial buffer on the ESP side."""
        if self.ser:
            try:
                # Send zeros for both linear and angular velocities
                zero_buff = struct.pack('=BBff', 36, 36, 0.0, 0.0)
                self.ser.write(zero_buff)
                self.get_logger().info("Initial zeros sent to clean the serial buffer")
                sleep(0.1)  # Allow some time for the ESP to process
            except serial.SerialException as e:
                self.get_logger().error(f"Error while sending initial zeros: {e}")

    def listener_callback(self, msg):
        """Send velocity data (linear and angular) from /cmd_vel topic to the ESP over serial."""
        linear_vel = msg.linear.x*(1/250)
        angular_vel = msg.angular.z*(1/10)
        #self.get_logger().info(f"Received /cmd_vel - linear_vel: {linear_vel}, angular_vel: {angular_vel}")

        # Pack and send the velocities over serial
        if self.ser:
            try:
                # Send linear velocity and angular velocity (floats)
                buff = struct.pack('=BBff', 36, 36, linear_vel, angular_vel)
                self.ser.write(buff)
                self.get_logger().debug("Velocity data sent to serial")
                sleep(0.1)
            except serial.SerialException as e:
                self.get_logger().error(f"Serial communication error while sending: {e}")

    def read_serial_data(self):
        """Periodically read wheel speeds from the serial port and publish to /wheel_speed."""
        if self.ser:
            try:
                # Ensure the buffer is large enough to read (2 header + 4 floats = 18 bytes)
                while self.ser.in_waiting >= 18:
                    data = self.ser.read(18)
                    #self.get_logger().info(f"Raw data received: {list(data)}")

                    # Look for the correct header (36, 36), and shift data until it's found
                    header_index = self.find_valid_header(data)
                    if header_index != -1:
                        data = data[header_index:]

                        # Check if enough data remains after the header
                        if len(data) >= 18:
                            try:
                                # Unpack 4 floats: left_wheel_speed, right_wheel_speed, velocity, orientation
                                signals = struct.unpack('ffff', data[2:])
                                left_wheel_speed, right_wheel_speed, velocity, orientation = signals

                                # Log the unpacked data for debugging
                               # self.get_logger().info(f"Unpacked wheel speeds: Left={left_wheel_speed}, Right={right_wheel_speed}")
                               # self.get_logger().info(f"Unpacked velocity: {velocity}, orientation: {orientation}")

                                # Publish the wheel speeds to /wheel_speed topic
                                wheel_speed_msg = Float32MultiArray()
                                wheel_speed_msg.data = [left_wheel_speed, right_wheel_speed]
                                self.wheel_speed_publisher.publish(wheel_speed_msg)

                            except struct.error as e:
                                self.get_logger().error(f"Data unpacking error: {e}")
                                self.ser.reset_input_buffer()  # Reset buffer on unpacking error
                        else:
                            self.get_logger().warn("Not enough data after the header to unpack floats")
                    else:
                        self.get_logger().warn("Valid header not found, discarding data")
                        self.ser.reset_input_buffer()  # Reset buffer if header not found

            except serial.SerialException as e:
                self.get_logger().error(f"Serial communication error while receiving: {e}")
                self.reinitialize_serial()

    def find_valid_header(self, data):
        """Look for the valid 36, 36 header in the incoming data."""
        for i in range(len(data) - 1):
            if data[i] == 36 and data[i + 1] == 36:
                return i  # Found valid header, return its index
        return -1  # Header not found

    def reinitialize_serial(self):
        """Reinitialize serial communication in case of persistent errors."""
        self.get_logger().warn("Reinitializing serial connection due to persistent errors")
        if self.ser:
            self.ser.close()
        sleep(1)
        self.initialize_serial()

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()

    try:
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        serial_node.get_logger().info("Shutdown requested by user...")
    finally:
        if serial_node.ser and serial_node.ser.is_open:
            serial_node.ser.close()
            serial_node.get_logger().info("Serial port closed.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
