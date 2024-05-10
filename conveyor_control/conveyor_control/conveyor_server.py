#!/usr/bin/env python3
import rclpy
import time 
from rclpy.node import Node
from custom_msg.action import SpeedControl
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray



class ConveyorVelServerNode(Node): 
    def __init__(self):
        super().__init__("conveyoy_vel_server") 
        self.count_until_server = ActionServer(
            self, 
            SpeedControl,
            "conveyor_vel",
            execute_callback=self.execute_callback,)
        self.get_logger().info("Conveyour velocity control has been started")

        self.publisher_ = self.create_publisher(Float64MultiArray,"nekonote_conveyor/forward_position_controller/commands",10)

    def publish_msg(self):
        msg = Float64MultiArray()
        msg.data = [1.0, 2.0, 3.0]
        self.publisher_.publish(msg)
        

    def execute_callback(self,goal_handle: ServerGoalHandle):
        # get request from the goal 
        result = SpeedControl.Result()

        conveyor_vel = goal_handle.request.conveyor_vel 
        period = goal_handle.request.period
        
        self.get_logger().info("Execute the goal")
        self.get_logger().info("Speed: " + str(conveyor_vel))
        self.get_logger().info("Period: " + str(period))

        start_time = time.time()
         # Execute action

        while time.time() < start_time + period:
            
            self.get_logger().info("Time: " + str(time.time()))
            self.publish_msg()
            time.sleep(0.5)

        #one doe 
        goal_handle.succeed()


        # send the result
        result.conveyor_result = True
        return result
    


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorVelServerNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()