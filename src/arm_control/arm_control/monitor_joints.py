#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math
import os

class JointMonitorNode(Node):
    def __init__(self):
        super().__init__('joint_monitor_node')
        
        self.status = "unknown"
        self.log_filepath = "/home/danish/Amarnath/arm/joint_monitor.log"
        
        # Clear log file on startup
        if os.path.exists(self.log_filepath):
            os.remove(self.log_filepath)
            
        self.log("Joint Monitor Node started.")
        
        self.status_sub = self.create_subscription(
            String,
            '/arm/status',
            self.status_callback,
            10
        )
        
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

    def status_callback(self, msg: String):
        self.status = msg.data
        self.log(f"Status changed to: {self.status}")

    def joint_callback(self, msg: JointState):
        has_nan = False
        nan_joints = []
        for name, pos in zip(msg.name, msg.position):
            if math.isnan(pos):
                has_nan = True
                nan_joints.append(name)
                
        if has_nan:
            joint_str = ", ".join([f"{n}: {p}" for n, p in zip(msg.name, msg.position)])
            self.log(f"[NAN DETECTED] Status: {self.status} | NaNs in: {nan_joints} | All: {joint_str}")
        else:
            # Periodically log healthy states (every 100th message to avoid spam)
            if not hasattr(self, 'msg_count'):
                self.msg_count = 0
            self.msg_count += 1
            if self.msg_count % 100 == 0:
                joint_str = ", ".join([f"{n}: {p:.4f}" for n, p in zip(msg.name, msg.position)])
                self.log(f"[HEALTHY] Status: {self.status} | {joint_str}")

    def log(self, text: str):
        time_msg = self.get_clock().now().to_msg()
        timestamp = f"{time_msg.sec}.{time_msg.nanosec:09d}"
        log_line = f"[{timestamp}] {text}\n"
        print(log_line, end="")
        with open(self.log_filepath, "a") as f:
            f.write(log_line)

def main(args=None):
    rclpy.init(args=args)
    node = JointMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
