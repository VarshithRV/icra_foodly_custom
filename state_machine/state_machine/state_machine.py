#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msg.action import MotionPlan, SpeedControl
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from geometry_msgs.msg import PointStamped
import threading as td
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
import image_geometry
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import numpy as np

######### Crop values of the image #########
X1 = 125 # left
Y1 = 50  # top 
X2 = 60  # right
Y2 = 200 # bottom
############################################
MASK_THRESHOLD = 2 # mask will become slimmer if the value is decreased, smaller => more precise range(0, 255)
############################################

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
  

class ImageFeedbackNode(Node):
    def __init__(self):
        super().__init__('image_processor_node')
        
        self.get_logger().info("Image Processing Started")
        
        # Initialize a CvBridge to convert between ROS Image and OpenCV
        self.cv_bridge = CvBridge()
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_info = None
        self.depth_image = None
        self.color_image = None
        self.counter0 = 0
        self.counter1 = 0
        self.counter2 = 0

        # Create a TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a subscriber to receive Image messages
        self.color_subscriber = self.create_subscription(
            Image, '/chest_camera/color/image_raw', self.color_callback, 10
        )

        # Create a subscriber to receive Image messages
        self.depth_to_color_subscriber = self.create_subscription(
            Image, '/chest_camera/aligned_depth_to_color/image_raw', self.image_callback, 10
        )

        # Creating a subscriber for camera info
        self.camera_info_sub = self.create_subscription(
            CameraInfo,'/chest_camera/aligned_depth_to_color/camera_info',self.camera_info_callback,10
        )

        # Create a publisher to send PointStamped messages
        self.point_publisher0 = self.create_publisher(PointStamped, '/don0_centroid_3d', 10)
        self.point_publisher1 = self.create_publisher(PointStamped, '/don1_centroid_3d', 10)


    # function that checks if the image has changed given two sets of average rgb values
    def has_image_changed(self, avg_rgb1, avg_rgb2):
        if avg_rgb1 is None or avg_rgb2 is None:
            return False
        # Calculate the difference between the average RGB values
        diff = np.linalg.norm(avg_rgb1 - avg_rgb2)
        return diff > 10

    # function to get the average RGB values of the full image
    def get_avg_rgb(self):
        if self.color_image is None:
            return
        # get the average RGB values of the full image
        avg_rgb = np.mean(self.color_image, axis=(0, 1))
        return avg_rgb
        
    
    # Callback function for the Image message
    def color_callback(self, msg):
        if self.counter0 == 0:
            self.get_logger().info("Color Image Received")
        self.counter0 += 1
        self.color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.color_image = self.color_image[Y1:-Y2,X1:-X2]
        # write the image to check the cropping
        cv2.imwrite('color_image.png', self.color_image)


    # Callback function for the CameraInfo message
    def camera_info_callback(self, msg):
        self.camera_model.fromCameraInfo(msg)
        self.camera_info = msg
        if self.counter0 == 0:
            self.get_logger().info("Camera Info Received")
        self.counter0 += 1
    
    
    # projecting the pixel to 3d
    def project_pixel_to_3d(self, x, y):
        if self.depth_image is None:
            return  # Wait until depth image is received
        depth = self.depth_image[y, x]  # Convert to meters
        if np.isnan(depth) or depth == 0:
            self.get_logger().warn("Invalid depth at pixel ({}, {})".format(x, y))
            return
        # Project the 2D pixel to 3D point in the camera frame
        point_3d = self.camera_model.projectPixelTo3dRay((x, y))
        point_3d = np.array(point_3d) * depth  # Scale the ray by the depth
        return point_3d

    
    # Transformer
    def transform_point(self, point:PointStamped, target_frame):
        try:
            # Get the latest transform from point.header.frame_id to target_frame
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                point.header.frame_id,
                rclpy.time.Time()
            )

            # Transform the point
            transformed_point = tf2_geometry_msgs.do_transform_point(point, transform)
            return transformed_point

        except TransformException as ex:
            self.get_logger().error(f"Transform failed: {ex}")
            return None
        


    # Callback function for the Image message, finding the centroid of the DONs and the 3d point
    def image_callback(self, msg):
        timer_now = time.time()
        try:
            if self.counter1==0:
                self.get_logger().info("Depth Image Received")
            self.counter1+=1
            if self.camera_info is None:
                return
            
            # make a pass through depth image
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            normalized_img_parent = cv2.normalize(
                cv_image, None, 0, 255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U
            )
            # cv2.imwrite('normalized_parent.png', normalized_img_parent) # use this to check the cv_bridge conversion               
            # cropping the image here
            cv_image = cv_image[Y1:-Y2,X1:-X2]
            # normalize this once more
            normalized_img = cv2.normalize(
                cv_image, None, 0, 255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U
            )
            # cv2.imwrite('normalized_cropped.png', normalized_img) # use this to validate the CROP values
            
            # create a mask for the boundary of the DON
            mask1 = np.zeros(normalized_img.shape, dtype=np.uint8)
            mask2 = np.zeros(normalized_img.shape, dtype=np.uint8)
            # if y is less than 240, then mask1 is 1
            for x in range(0, normalized_img.shape[1]):
                for y in range(0, normalized_img.shape[0]):
                    if normalized_img[y][x] < MASK_THRESHOLD:
                        if y < (normalized_img.shape[0])/2:
                            mask1[y][x] = 255
                        else:
                            mask2[y][x] = 255
            # cv2.imwrite('mask.png', mask) # use this to validate the MASK_THRESHOLD value
            
            # get the centroid
            M = cv2.moments(mask1)
            cx1 = int(M["m10"] / M["m00"]) + X1
            cy1 = int(M["m01"] / M["m00"]) + Y1
            M = cv2.moments(mask2)
            cx2 = int(M["m10"] / M["m00"]) + X1
            cy2 = int(M["m01"] / M["m00"]) + Y1
            
            # project the centroid to 3d
            point_3d1 = self.project_pixel_to_3d(cx1, cy1)
            point_3d2 = self.project_pixel_to_3d(cx2, cy2)
            
            point0= PointStamped()
            point0.header = msg.header
            point0.point.x = point_3d2[0]
            point0.point.y = point_3d2[1]
            point0.point.z = point_3d2[2]
            point0 = self.transform_point(point0, 'base_link')
            
            point1 = PointStamped()
            point1.header = msg.header
            point1.point.x = point_3d1[0]
            point1.point.y = point_3d1[1]
            point1.point.z = point_3d1[2]
            point1 = self.transform_point(point1, 'base_link')

            if self.counter2 == 0:
                self.get_logger().info(f"Publishing to /don0_centroid_3d and /don1_centroid_3d topics, Type : PointStamped")
            
            self.counter2 += 1
            self.point_publisher0.publish(point0)
            self.point_publisher1.publish(point1)

        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def spin_node(node):
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)


    ######################################### define client #####################################
    conveyor_node = ConveyorClient() 
    left_arm_client = LeftArmClient()
    right_arm_client = RightArmClient()
    perception_client = PerceptionClient()
    feedback_client = ImageFeedbackNode()

    perception_client_thread = td.Thread(target=rclpy.spin, args=(perception_client,))
    perception_client_thread.start()
    thread = td.Thread(target=spin_node, args=(feedback_client,), daemon=True)
    thread.start()

    print("All nodes are spinning successfully, starting the operation now")

    try :
        while True : 
            x = input("Enter to start the iteration, 'x' to stop")
            if x=="x":
                break

            # Conveyor belt
            print("Moving DONS to the Left arm")
            # conveyor_node.send_goal(CONVEYOR_SPEED,CONVEYOR_INTERVAL_1) 
            # conveyor_node.get_logger().info("Waiting for result from converyor belt")
            # while not (conveyor_node.result_status):
            #     pass
            # conveyor_node.result_status = False

            time.sleep(0.2)# give time for perception

            while True :
                avg_rgb1 = feedback_client.get_avg_rgb()

                # # Left arm DON 0
                # print("Filling DON0")
                # left_arm_client.send_goal(
                #     perception_client.don0_point.point.x,
                #     perception_client.don0_point.point.y,
                #     perception_client.don0_point.point.z
                #     )
                # while not (left_arm_client.result_status):
                #     # rclpy.spin_once(left_arm_client)
                #     # left_arm_client.get_logger().info('Wait for result from left arm motion')
                #     pass
                # left_arm_client.result_status = False

                # # Left arm DON 1
                # print("Filling DON1")
                # left_arm_client.send_goal(
                #     perception_client.don1_point.point.x,
                #     perception_client.don1_point.point.y,
                #     perception_client.don1_point.point.z
                #     )
                # while not (left_arm_client.result_status):
                #     # rclpy.spin_once(left_arm_client)
                #     # left_arm_client.get_logger().info('Wait for result from left arm motion')
                #     pass
                # left_arm_client.result_status = False

                if feedback_client.has_image_changed(avg_rgb1, feedback_client.get_avg_rgb()):
                    break

            # Conveyor belt
            # print("Moving DONS to the Right arm")
            # conveyor_node.send_goal(CONVEYOR_SPEED,CONVEYOR_INTERVAL_2) 
            # conveyor_node.get_logger().info("Waiting for result from converyor belt")
            # while not (conveyor_node.result_status):
            #     pass
            # conveyor_node.result_status = False

            time.sleep(0.2)# give time for perception

            # Right arm DON0
            print("Filling DON0")
            right_arm_client.send_goal(
                perception_client.don0_point.point.x,
                perception_client.don0_point.point.y,
                perception_client.don0_point.point.z + MOTION_PLANNING_OFFSET_Z
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
                perception_client.don1_point.point.z + MOTION_PLANNING_OFFSET_Z
                )
            right_arm_client.get_logger().info('Wait for result from left arm motion')
            while not (right_arm_client.result_status):
                pass
            right_arm_client.result_status = False

            # Conveyor belt
            # print("Moving DONS to unloading position")
            # conveyor_node.send_goal(CONVEYOR_SPEED,CONVEYOR_INTERVAL_2) 
            # conveyor_node.get_logger().info("Waiting for result from converyor belt")
            # while not (conveyor_node.result_status):
            #     pass
            # conveyor_node.result_status = False

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