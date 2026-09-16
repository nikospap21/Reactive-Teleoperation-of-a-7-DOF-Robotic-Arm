import time
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class RVizVisualizer:
    def __init__(self, node_name="myarm_teleop_visualizer"):
        self.enabled = ROS2_AVAILABLE
        if not self.enabled:
            print("[WARN] ROS 2 (rclpy) δεν βρέθηκε. Η 3D απεικόνιση στο RViz είναι απενεργοποιημένη.")
            return

        if not rclpy.ok():
            rclpy.init()

        self.node = rclpy.create_node(node_name)
        self.publisher = self.node.create_publisher(JointState, "/joint_states", 10)

        # Τα standard joint names για το MyArm 300 Pi στο URDF
        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7"
        ]
        print("[+] ROS 2 Visualizer ενεργός -> Δημοσίευση στο /joint_states")

    def publish_joints(self, q_current):
        """
        q_current: numpy array ή list 7 γωνιών σε radians
        """
        if not self.enabled:
            return

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(val) for val in q_current]

        self.publisher.publish(msg)
        # SpinOnce χωρίς blocking για να εξυπηρετηθεί το ROS loop
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def shutdown(self):
        if self.enabled and rclpy.ok():
            self.node.destroy_node()
            rclpy.shutdown()