import struct
import fcntl
import os
import sys
import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Bool  # type: ignore
from sensor_msgs.msg import JointState  # type: ignore

class DirectXboxTeleop(Node):
    def __init__(self):
        super().__init__('direct_xbox_teleop')
        self.gripper_pub = self.create_publisher(Bool, '/suction_gripper_state', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Στόχευση στο πραγματικό Xbox controller
        dev_path = '/dev/input/js2'
        try:
            self.js_file = open(dev_path, 'rb')
            fd = self.js_file.fileno()
            flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
            self.get_logger().info(f"Σύνδεση επιτυχής στο {dev_path}!")
        except Exception as e:
            self.get_logger().error(f"Σφάλμα ανοίγματος {dev_path}: {e}")
            sys.exit()

        self.gripper_state = False
        self.create_timer(0.01, self.poll_joystick)

    def poll_joystick(self):
        try:
            while True:
                data = self.js_file.read(8)
                if not data:
                    break
                
                time, value, type_, number = struct.unpack('IhBB', data)
                type_clean = type_ & ~0x80

                # Button Events (Type 1)
                if type_clean == 1:
                    if value == 1: # Button Press
                        self.get_logger().info(f"Button {number} Pressed!")
                        
                        if number == 0: # Button A -> Grasp
                            self.gripper_state = True
                            self.publish_gripper("GRASP")
                        elif number == 1: # Button B -> Release
                            self.gripper_state = False
                            self.publish_gripper("RELEASE")

                # Axis Events (Type 2 - Μοχλοί)
                elif type_clean == 2:
                    if abs(value) > 10000: # Deadzone filter
                        self.get_logger().info(f"Axis {number} Value: {value}")

        except BlockingIOError:
            pass

    def publish_gripper(self, state_str):
        msg = Bool()
        msg.data = self.gripper_state
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"Vacuum Gripper: {state_str}")

def main(args=None):
    rclpy.init(args=args)
    node = DirectXboxTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
