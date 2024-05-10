#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import threading as td
import time

# Node containing all the clients for the central node
class CentralClient(Node):
    def __init__(self):
        
        super().__init__('central_client')
        self.get_logger().info('Central Client Node Started')
        self.get_logger().info("Changes")
        
        # Subscriber for don0 Pose
        self.don0_subscription = self.create_subscription(
            PointStamped, '/don0_centroid_3d', self.don0_pose_cb, 50
        )
        
        # Subscriber for don1 Pose
        self.don1_subscription = self.create_subscription(
            PointStamped, '/don1_centroid_3d', self.don1_pose_cb, 50
        )
        
        # Pose variables to store messages
        self.don0_pose = None
        self.don1_pose = None

        # Add more clients or subscriber or servers here.
        #################################################
    
    def don0_pose_cb(self, msg: PointStamped):
        self.don0_pose = msg

    def don1_pose_cb(self, msg: PointStamped):
        self.don1_pose = msg

def main(args=None):
    rclpy.init(args=args)
    
    central_client = CentralClient()
    
    thread = td.Thread(target=rclpy.spin, args=(central_client,))
    thread.start() # start the spinning in a different thread

    ########################## WE HAVE CONTROL FROM HERE #################################
    while rclpy.ok():
        if central_client.don0_pose is not None:
            central_client.get_logger().info(f"don0_pose: {central_client.don0_pose}")
            time.sleep(0.5)
    ########################## WE HAVE CONTROL TILL HERE #################################
    
    
    thread.join() # waiting for the thread to finish( it actually won't finish because rclpy.spin() will run forever)
    central_client.destroy_node()
    rclpy.shutdown()

main()