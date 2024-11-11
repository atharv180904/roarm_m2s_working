#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time

class ServoControl:
    def __init__(self, port, baudrate=1000000):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)

    def __del__(self):
        self.ser.close()

    def calculate_checksum(self, data):
        return (~sum(data)) & 0xFF

    def send_command(self, command):
        self.ser.write(command)
        time.sleep(0.05)

    def move_to_position(self, id, position, speed=300):
        command = bytes([0xFF, 0xFF, id, 0x09, 0x03, 0x2A])
        command += position.to_bytes(2, byteorder='little')
        command += bytes([0x00, 0x00])  # Add necessary bytes
        command += speed.to_bytes(2, byteorder='little')
        checksum = self.calculate_checksum(command[2:])
        command += bytes([checksum])
        self.send_command(command)
        print(f"Command sent to move servo {id} to position {position} at speed {speed}")

class JointStateListener(Node):
    def __init__(self, port):
        super().__init__('driver')
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.subscription 
        self.last_time = self.get_clock().now().nanoseconds
        self.updated_angles = [0.0] * 4  
        self.updated_positions = [0] * 4  
        self.servo_control = ServoControl(port) 

    def joint_state_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds
        
        if (current_time - self.last_time) / 1e9 > 0.001: #100Hz 
            self.last_time = current_time
            if len(msg.position) > 0:
                self.updated_angles[1] = (((msg.position[1]) + 4.71238898) % 6.28)
            if len(msg.position) > 1:
                self.updated_angles[0] = ((msg.position[0] + 3.141592654) % 6.28)
            if len(msg.position) > 2:
                self.updated_angles[2] = msg.position[2] + 3.141592654  
            for i in range(3):  
                angle_in_degrees = self.updated_angles[i] * (180.0 / 3.141592654)  
                normalized_angle = angle_in_degrees % 360  
                self.updated_positions[i] = angle_to_position(normalized_angle)
                
            servo_ids = [2, 1, 3]
            for i in range(3):
                self.servo_control.move_to_position(servo_ids[i], self.updated_positions[i])
            
            # Add the line to keep servo with ID 4 at position 573
            self.servo_control.move_to_position(4, 200)
                
            joint_positions_msg = ", ".join(
                f"{joint}: {self.updated_positions[i]}" 
                for i, joint in enumerate(msg.name[:3])
            )
            
            joint_positions_msg += f", {msg.name[3]}: {self.updated_positions[3]}" if len(msg.name) > 3 else ""
            self.get_logger().info(f'Joint positions: {joint_positions_msg}')
 
def angle_to_position(angle):
    return int((angle % 360) * 4096 / 360)

def main(args=None):
    rclpy.init(args=args)
    available_port = '/dev/ttyACM0'
    joint_state_listener = JointStateListener(available_port)
    rclpy.spin(joint_state_listener)
    joint_state_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()