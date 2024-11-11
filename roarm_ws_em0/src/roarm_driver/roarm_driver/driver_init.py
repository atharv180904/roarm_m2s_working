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
        time.sleep(0.05)  # Give the servo some time to process

    def read_response(self):
        return self.ser.read(100)

    def disable_eeprom_lock(self, id):
        command = bytes([0xFF, 0xFF, id, 0x04, 0x03, 0x37, 0x00])
        checksum = self.calculate_checksum(command[2:])
        command += bytes([checksum])
        self.send_command(command)
        print("EEPROM lock disabled")

    def enable_eeprom_lock(self, id):
        command = bytes([0xFF, 0xFF, id, 0x04, 0x03, 0x37, 0x01])
        checksum = self.calculate_checksum(command[2:])
        command += bytes([checksum])
        self.send_command(command)
        print("EEPROM lock enabled")

    def change_id(self, current_id, new_id):
        # Disable EEPROM lock
        self.disable_eeprom_lock(current_id)
        
        # Change ID
        command = bytes([0xFF, 0xFF, current_id, 0x04, 0x03, 0x05, new_id])
        checksum = self.calculate_checksum(command[2:])
        command += bytes([checksum])
        self.send_command(command)
        print(f"Command sent to change ID from {current_id} to {new_id}")
        
        # Wait a moment for the change to take effect
        time.sleep(0.1)
        
        # Enable EEPROM lock
        self.enable_eeprom_lock(new_id)
        
        print(f"ID changed from {current_id} to {new_id} and saved to EEPROM")

    def ping(self, id):
        command = bytes([0xFF, 0xFF, id, 0x02, 0x01])
        checksum = self.calculate_checksum(command[2:])
        command += bytes([checksum])
        self.send_command(command)
        response = self.read_response()
        if response:
            print(f"Servo {id} responded: {response.hex()}")
        else:
            print(f"No response from servo {id}")

    def move_to_position(self, id, position, speed=1000):
        command = bytes([0xFF, 0xFF, id, 0x09, 0x03, 0x2A])
        command += position.to_bytes(2, byteorder='little')
        command += bytes([0x00, 0x00])  # Time (0 for continuous rotation)
        command += speed.to_bytes(2, byteorder='little')
        checksum = self.calculate_checksum(command[2:])
        command += bytes([checksum])
        self.send_command(command)
        print(f"Command sent to move servo {id} to position {position} at speed {speed}")

    def read_position(self, id):
        command = bytes([0xFF, 0xFF, id, 0x04, 0x02, 0x38, 0x02])
        checksum = self.calculate_checksum(command[2:])
        command += bytes([checksum])
        self.send_command(command)
        response = self.read_response()
        if response and len(response) >= 8:
            position = int.from_bytes(response[5:7], byteorder='little')
            print(f"Current position of servo {id}: {position}")
            return position
        else:
            print(f"Failed to read position from servo {id}")
            return None

def main():
    # Replace 'COM3' with your actual serial port
    servo = ServoControl('/dev/ttyACM0')

    while True:
        print("\n1. Change ID")
        print("2. Ping servo")
        print("3. Move servo")
        print("4. Read position")
        print("5. Exit")
        choice = input("Enter your choice: ")

        if choice == '1':
            current_id = int(input("Enter current ID: "))
            new_id = int(input("Enter new ID (1-253): "))
            servo.change_id(current_id, new_id)
        elif choice == '2':
            id = int(input("Enter servo ID to ping: "))
            servo.ping(id)
        elif choice == '3':
            id = int(input("Enter servo ID: "))
            position = int(input("Enter position (0-4095): "))
            speed = int(input("Enter speed (1-1023): "))
            servo.move_to_position(id, position, speed)
        elif choice == '4':
            id = int(input("Enter servo ID: "))
            servo.read_position(id)
        elif choice == '5':
            break
        else:
            print("Invalid choice. Please try again.")

if __name__ == "__main__":
    main()