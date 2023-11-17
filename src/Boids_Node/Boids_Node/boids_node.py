#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from px4_msgs.msg import VehicleLocalPosition
from .Boids_Simulator.simulation import Simulation
from boid_msg.msg import BoidStatus

class BoidsNode(Node):

    def __init__(self):
        
        super().__init__('boids_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )

        self.x_drone = 0.0
        self.y_drone = 0.0
        self.timer_period = 0.02  # seconds

        """Initialise the Simulator"""
        window = (1000, 1000)
        margin =   420
        self.simulation = Simulation(window, margin, 50)
        self.simulation.graphic_interface()
        
        """ Various Callbacks """
        self.position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_callback, qos_profile)
        self.simulation_callback = self.create_timer(self.timer_period, self.simulation_callback)
    
        self.boids_pub = self.create_publisher(BoidStatus, '/boids/positions', qos_profile)
        self.boids_timer = self.create_timer(self.timer_period, self.boids_callback)


    def position_callback(self, msg: VehicleLocalPosition):
        self.x_robot = msg.x*3 + 500
        self.y_robot = msg.y*3 + 500

    def simulation_callback(self):
        self.simulation.update_animation(self.x_robot, self.y_robot)

    def boids_callback(self):        
        msg = BoidStatus()
        msg.boidpositions = self.simulation.data_formatting()
        self.boids_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BoidsNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()




