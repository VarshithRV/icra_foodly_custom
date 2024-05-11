#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msg.action import MotionPlan, SpeedControl
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from geometry_msgs.msg import PointStamped
import threading as td
import time

####### CONVERYOR TUNING PARAMETERS #######
CONVEYOR_SPEED = 0.5 #m/s
CONVEYOR_INTERVAL_1 = 3.0 #seconds
CONVEYOR_INTERVAL_2 = 5.0 #seconds
CONVEYOR_INTERVAL_3 = 3.0 #seconds
MOTION_PLANNING_OFFSET_Z = 0.05 #METERS
####### CONVERYOR TUNING PARAMETERS #######

class ConveyorClient(Node): 
    def __init__(self):
        super().__init__("conveyor_control_client") 
        self.count_until_client = ActionClient(
            self, 
            SpeedControl,
            "conveyor_vel")
        
        self.result_status = False


    def send_goal(self, conveyor_vel, period):
        #Wait for server
        self.count_until_client.wait_for_server()
        
        # Create goal
        goal = SpeedControl.Goal()
        goal.conveyor_vel = conveyor_vel
        goal.period = period


        #send the goal 
        self.get_logger().info("Sending goal")
        self.count_until_client.\
            send_goal_async(goal).\
                add_done_callback(self.goal_response_callback) # send_goal() will block until it receive the result # this callback receive that goal is accepted or rejected


    def goal_response_callback(self, future):
        self.goal_handle_:ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal is accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback) 

    # Send request for the response from the server
    def goal_result_callback(self,future):
        result = future.result().result
        if (result.conveyor_result):
            self.result_status = True
            self.get_logger().info("Converyor belt done ... ")


class LeftArmClient(Node): 
    def __init__(self):
        super().__init__("left_arm_client") 
        self.count_until_client = ActionClient(
            self, 
            MotionPlan,
            "left_arm_motion_plan")
        
        self.result_status = False
        
    def send_goal(self, x, y, z):
        #Wait for server
        self.count_until_client.wait_for_server()
        
        # Create goal
        goal = MotionPlan.Goal()
        goal.bowl_x = x
        goal.bowl_y = y
        goal.bowl_z = z

        #send the goal 
        self.get_logger().info("Sending goal")
        self.count_until_client.\
            send_goal_async(goal).\
                add_done_callback(self.goal_response_callback) 


    def goal_response_callback(self, future):
        self.goal_handle_:ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal is accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback) 
        else:
            self.get_logger().warn("Goal got rejected")

    # Send request for the response from the server
    def goal_result_callback(self,future):
        result = future.result().result
        self.result_status = True

        if (result.motion_result):
            self.get_logger().info("left arm done moving")


class RightArmClient(Node): 
    def __init__(self):
        super().__init__("right_arm_client") 
        self.count_until_client = ActionClient(
            self, 
            MotionPlan,
            "right_arm_motion_plan")
        self.result_status = False
        
    def send_goal(self, x, y, z):
        #Wait for server
        self.count_until_client.wait_for_server()
        
        # Create goal
        goal = MotionPlan.Goal()
        goal.bowl_x = x
        goal.bowl_y = y
        goal.bowl_z = z

        #send the goal 
        self.get_logger().info("Sending goal")
        self.count_until_client.\
            send_goal_async(goal).\
                add_done_callback(self.goal_response_callback) # send_goal() will block until it receive the result # this callback receive that goal is accepted or rejected


    def goal_response_callback(self, future):
        self.goal_handle_:ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal is accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback) 
        else:
            self.get_logger().warn("Goal got rejected")

    # Send request for the response from the server
    def goal_result_callback(self,future):
        result = future.result().result
        self.result_status = True
        if (result.motion_result):
            self.get_logger().info("left arm done moving")


class PerceptionClient(Node):
    def __init__(self):
        
        super().__init__('central_client')
        
        # Subscriber for don0 Pose
        self.don0_subscription = self.create_subscription(
            PointStamped, '/don0_centroid_3d', self.don0_point_cb, 50
        )
        
        # Subscriber for don1 Pose
        self.don1_subscription = self.create_subscription(
            PointStamped, '/don1_centroid_3d', self.don1_point_cb, 50
        )
        
        # Pose variables to store messages
        self.don0_point = None
        self.don1_point = None

        # Add more clients or subscriber or servers here.
        #################################################
    
    def don0_point_cb(self, msg: PointStamped):
        self.don0_point = msg

    def don1_point_cb(self, msg: PointStamped):
        self.don1_point = msg
  

def main(args=None):
    rclpy.init(args=args)


    ######################################### define client #####################################
    conveyor_node = ConveyorClient() 
    left_arm_client = LeftArmClient()
    right_arm_client = RightArmClient()
    perception_client = PerceptionClient()

    # conveyor_node_thread = td.Thread(target=rclpy.spin, args=(conveyor_node))
    perception_client_thread = td.Thread(target=rclpy.spin, args=(perception_client,))
    # left_arm_client_thread = td.Thread(target=rclpy.spin, args=(left_arm_client,))
    # right_arm_client_thread = td.Thread(target=rclpy.spin, args=(right_arm_client,))

    # conveyor_node_thread.start()
    perception_client_thread.start()
    # left_arm_client_thread.start()
    # right_arm_client_thread.start()

    print("All nodes are spinning successfully, starting the operation now")

    try :
        while True : 
            x = input("Enter to start the iteration, 'x' to stop")
            if x=="x":
                break

            # Conveyor belt
            print("Moving DONS to the Left arm")
            conveyor_node.send_goal(CONVEYOR_SPEED,CONVEYOR_INTERVAL_1) 
            conveyor_node.get_logger().info("Waiting for result from converyor belt")
            while not (conveyor_node.result_status):
                pass
            conveyor_node.result_status = False

            time.sleep(0.2)# give time for perception

            # Left arm DON 0
            print("Filling DON0")
            left_arm_client.send_goal(
                perception_client.don0_point.point.x,
                perception_client.don0_point.point.y,
                perception_client.don0_point.point.z
                )
            while not (left_arm_client.result_status):
                # rclpy.spin_once(left_arm_client)
                # left_arm_client.get_logger().info('Wait for result from left arm motion')
                pass
            left_arm_client.result_status = False

            # Left arm DON 1
            print("Filling DON1")
            left_arm_client.send_goal(
                perception_client.don1_point.point.x,
                perception_client.don1_point.point.y,
                perception_client.don1_point.point.z
                )
            while not (left_arm_client.result_status):
                # rclpy.spin_once(left_arm_client)
                # left_arm_client.get_logger().info('Wait for result from left arm motion')
                pass
            left_arm_client.result_status = False

            # Conveyor belt
            print("Moving DONS to the Right arm")
            conveyor_node.send_goal(CONVEYOR_SPEED,CONVEYOR_INTERVAL_2) 
            conveyor_node.get_logger().info("Waiting for result from converyor belt")
            while not (conveyor_node.result_status):
                pass
            conveyor_node.result_status = False

            time.sleep(0.2)# give time for perception

            # Right arm DON0
            print("Filling DON0")
            right_arm_client.send_goal(
                perception_client.don0_point.point.x,
                perception_client.don0_point.point.y,
                perception_client.don0_point.point.z
                )
            right_arm_client.get_logger().info('Wait for result from left arm motion')
            while not (right_arm_client.result_status):
                pass
            right_arm_client.result_status = False

            # Right arm DON1
            print("Filling DON1")
            right_arm_client.send_goal(
                perception_client.don1_point.point.x,
                perception_client.don1_point.point.y,
                perception_client.don1_point.point.z
                )
            right_arm_client.get_logger().info('Wait for result from left arm motion')
            while not (right_arm_client.result_status):
                pass
            right_arm_client.result_status = False

            # Conveyor belt
            print("Moving DONS to unloading position")
            conveyor_node.send_goal(CONVEYOR_SPEED,CONVEYOR_INTERVAL_2) 
            conveyor_node.get_logger().info("Waiting for result from converyor belt")
            while not (conveyor_node.result_status):
                pass
            conveyor_node.result_status = False

    except  KeyboardInterrupt:
        print("Keyboard Interrupt detected, exiting")
    
    finally: 
        conveyor_node.destroy_node()
        left_arm_client.destroy_node()
        right_arm_client.destroy_node()
        perception_client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()