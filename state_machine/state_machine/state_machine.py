#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msg.action import MotionPlan, SpeedControl
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from geometry_msgs.msg import PointStamped
import threading as td
import time


class CoveyorClient(Node): 
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

# Node containing all the clients for the central node
class PerceptionClient(Node):
    def __init__(self):
        
        super().__init__('central_client')
        
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


    ######################################### define client #####################################
    coveyor_node = CoveyorClient() 
    left_arm_client = LeftArmClient()
    right_arm_client = RightArmClient()
    perception_client = PerceptionClient()


    # ##################################### Control Conveyor Belt ##################################
    # coveyor_node.send_goal(0.5,3.0)  #Both must be float
    # while not (coveyor_node.result_status):
    #     rclpy.spin_once(coveyor_node)
    #     coveyor_node.get_logger().info('Wait for result from conveyor belt')
    # coveyor_node.result_status = False
    # ##############################################################################################



    ####################################### Left arm control #####################################
    left_arm_client.send_goal(0.25,0.25,-0.1)
    while not (left_arm_client.result_status):
        rclpy.spin_once(left_arm_client)
        left_arm_client.get_logger().info('Wait for result from left arm motion')
    left_arm_client.result_status = False
    ##############################################################################################



    # ####################################### Right arm control #####################################
    # right_arm_client.send_goal(0.25,-0.2,0.1)
    # while not (right_arm_client.result_status):
    #     rclpy.spin_once(right_arm_client)
    #     right_arm_client.get_logger().info('Wait for result from right arm motion')
    # right_arm_client.result_status = False
    # ##############################################################################################



    # thread = td.Thread(target=rclpy.spin, args=(perception_client,))
    # thread.start() # start the spinning in a different thread

    # ########################## WE HAVE CONTROL FROM HERE #################################
    # while rclpy.ok():
    #     if perception_client.don0_pose is not None:
    #         perception_client.get_logger().info(f"don0_pose: {perception_client.don0_pose}")
    #         time.sleep(0.5)
    # ########################## WE HAVE CONTROL TILL HERE #################################
    
    
    # thread.join() # waiting for the thread to finish( it actually won't finish because rclpy.spin() will run forever)
    # perception_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()