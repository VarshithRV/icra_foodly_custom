#include <cmath>

#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "pose_presets.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_msg/action/motion_plan.hpp"

using MotionPlan = custom_msg::action::MotionPlan;
using MotionPlanGoalHandle = rclcpp_action::ServerGoalHandle<MotionPlan>;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using namespace std::placeholders;


class LeftArServerNode : public rclcpp::Node 
{
public:
    LeftArServerNode() : Node("left_arm_server") 
    {
        left_arm_server_ = rclcpp_action::create_server<MotionPlan>(
            this,
            "left_arm_motion_plan",
            std::bind(&LeftArServerNode::goal_callback, this, _1,_2),
            std::bind(&LeftArServerNode::cancel_callback, this, _1),
            std::bind(&LeftArServerNode::handle_accepted_callback, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "Left arm server has been started ");

    }

    bool goal_status = false;



private:

  rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MotionPlan::Goal> goal){
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<MotionPlanGoalHandle> goal_handle){
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }


    //  When the goal is accepted 
    void handle_accepted_callback(
        const std::shared_ptr<MotionPlanGoalHandle> goal_handle){
        RCLCPP_INFO(this->get_logger(),"Executing the motion planning in the left arm");
        // call the execution function
        execute_goal(goal_handle);
    }


    void execute_goal(const std::shared_ptr<MotionPlanGoalHandle> goal_handle){
        // Get goal variable 
        double bowl_x  = goal_handle->get_goal()->bowl_x;
        double bowl_y  = goal_handle->get_goal()->bowl_y;
        double bowl_z  = goal_handle->get_goal()->bowl_z;


        // Execute the action

        RCLCPP_INFO(this->get_logger(), "%f", bowl_x);
        RCLCPP_INFO(this->get_logger(), "%f", bowl_y);
        RCLCPP_INFO(this->get_logger(), "%f", bowl_z);

        this->goal_status = true;
        //set the final state and return result 
        
        auto result = std::make_shared<MotionPlan::Result>();
        result->motion_result = true ; 
        goal_handle->succeed(result);

    }

    rclcpp_action::Server<MotionPlan>::SharedPtr left_arm_server_;


};





int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Rate loop_rate(0.25);

    auto node = std::make_shared<LeftArServerNode>(); 


    // Declare nodes
    auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);

    // For current state monitor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_arm_node);
    executor.add_node(node);
    std::thread([&executor]() {executor.spin();}).detach();


    MoveGroupInterface move_group_arm(move_group_arm_node, "l_arm_group");
    move_group_arm.setMaxVelocityScalingFactor(0.5);  // Set 0.0 ~ 1.0
    move_group_arm.setMaxAccelerationScalingFactor(0.2);  // Set 0.0 ~ 1.0
    auto arm_joint_values = move_group_arm.getCurrentJointValues();

    while (true) { 


        if (node->goal_status) {

            RCLCPP_INFO(rclcpp::get_logger("Motion Planning"),"Start Motion Planning");
            move_group_arm.setNamedTarget("l_arm_init_pose");
            move_group_arm.move();


            arm_joint_values[0] = angles::from_degrees(0);
            arm_joint_values[1] = angles::from_degrees(90.0);
            arm_joint_values[2] = angles::from_degrees(0);
            arm_joint_values[3] = angles::from_degrees(-109.0);
            arm_joint_values[4] = angles::from_degrees(0.0);
            arm_joint_values[5] = angles::from_degrees(104.0);
            arm_joint_values[6] = angles::from_degrees(0.0);

            move_group_arm.setJointValueTarget(arm_joint_values);
            move_group_arm.move();

            node->goal_status = false;

        }

        RCLCPP_INFO(rclcpp::get_logger("Result from action"),"Waiting for Action request");
        loop_rate.sleep();

    }





    RCLCPP_INFO(rclcpp::get_logger("Node"),"This node is shutting down...");
    rclcpp::shutdown();

    return 0;
}