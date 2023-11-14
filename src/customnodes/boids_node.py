#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition



class BoidsNode(Node):



    def __init__(self):
        
        self.x_robot = 0.0
        self.y_robot = 0.0


        super().__init__('boids_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )
        self.position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_callback, qos_profile)

    def position_callback(self, msg: VehicleLocalPosition):
        #self.get_logger().info('I heard: "%s"' % msg)
        self.x_robot = msg.x
        self.y_robot = msg.y
        

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BoidsNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()




