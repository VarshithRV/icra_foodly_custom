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


class RightArServerNode : public rclcpp::Node 
{
public:
    RightArServerNode() : Node("right_arm_server") 
    {
        right_arm_server_ = rclcpp_action::create_server<MotionPlan>(
            this,
            "right_arm_motion_plan",
            std::bind(&RightArServerNode::goal_callback, this, _1,_2),
            std::bind(&RightArServerNode::cancel_callback, this, _1),
            std::bind(&RightArServerNode::handle_accepted_callback, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "Right arm server has been started ");

    }

    bool goal_status = false;
    double bowl_x ;
    double bowl_y ;
    double bowl_z ;

    bool motion_plan = true;





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
        RCLCPP_INFO(this->get_logger(),"Executing the motion planning in the right arm");
        // call the execution function
        execute_goal(goal_handle);
    }


    void execute_goal(const std::shared_ptr<MotionPlanGoalHandle> goal_handle){
        // Get goal variable 
        this->bowl_x  = goal_handle->get_goal()->bowl_x;
        this->bowl_y  = goal_handle->get_goal()->bowl_y;
        this->bowl_z  = goal_handle->get_goal()->bowl_z;
        rclcpp::Rate loop_rate(0.75);

        this->goal_status = true;

        while (this->motion_plan){
            RCLCPP_INFO(this->get_logger(),"Arm is executing ... ");
            loop_rate.sleep();this->goal_status = true;
        }

        this->motion_plan = true;
        
        //set the final state and return result 
        
        auto result = std::make_shared<MotionPlan::Result>();
        result->motion_result = true ; 
        RCLCPP_INFO(this->get_logger(),"Right arm action is done");
        goal_handle->succeed(result);
    }

    rclcpp_action::Server<MotionPlan>::SharedPtr right_arm_server_;


};





int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Rate loop_rate(0.25);

    auto node = std::make_shared<RightArServerNode>(); 

    
    // Declare nodes
    auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);

    // For current state monitor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_arm_node);
    executor.add_node(node);
    std::thread([&executor]() {executor.spin();}).detach();


    MoveGroupInterface move_group_arm(move_group_arm_node, "r_arm_group");
    move_group_arm.setMaxVelocityScalingFactor(0.5);  // Set 0.0 ~ 1.0
    move_group_arm.setMaxAccelerationScalingFactor(0.2);  // Set 0.0 ~ 1.0
    auto arm_joint_values = move_group_arm.getCurrentJointValues();

    while (true) { 
        if (node->goal_status) {

            RCLCPP_INFO(rclcpp::get_logger("Motion Planning"), "%f",  node->bowl_x);
            RCLCPP_INFO(rclcpp::get_logger("Motion Planning"), "%f",  node->bowl_y);
            RCLCPP_INFO(rclcpp::get_logger("Motion Planning"), "%f",  node->bowl_z);

            RCLCPP_INFO(rclcpp::get_logger("Motion Planning"),"Start Motion Planning");
            
            
            // --------- > Standby <-------------

            // set joint value 
            arm_joint_values[0] = angles::from_degrees(0);
            arm_joint_values[1] = angles::from_degrees(-90.0);
            arm_joint_values[2] = angles::from_degrees(0);
            arm_joint_values[3] = angles::from_degrees(110.0);
            arm_joint_values[4] = angles::from_degrees(0.0);
            arm_joint_values[5] = angles::from_degrees(-106.0);
            arm_joint_values[6] = angles::from_degrees(0.0);

            move_group_arm.setJointValueTarget(arm_joint_values);
            move_group_arm.move();




            // --------- > PrepPick <------------


            arm_joint_values[0] = angles::from_degrees(45.0);
            arm_joint_values[1] = angles::from_degrees(-90.0);
            arm_joint_values[2] = angles::from_degrees(0.0);
            arm_joint_values[3] = angles::from_degrees(50.0);
            arm_joint_values[4] = angles::from_degrees(0.0);
            arm_joint_values[5] = angles::from_degrees(60.0);
            arm_joint_values[6] = angles::from_degrees(0.0);

            move_group_arm.setJointValueTarget(arm_joint_values);
            move_group_arm.move();




            // =============> Pick <=============
            arm_joint_values[0] = angles::from_degrees(53.0);
            arm_joint_values[1] = angles::from_degrees(-90.0);
            arm_joint_values[2] = angles::from_degrees(0.0);
            arm_joint_values[3] = angles::from_degrees(24.0);
            arm_joint_values[4] = angles::from_degrees(0.0);
            arm_joint_values[5] = angles::from_degrees(51.0);
            arm_joint_values[6] = angles::from_degrees(0.0);

            move_group_arm.setJointValueTarget(arm_joint_values);
            move_group_arm.move();



            // --------- > PrepPick <------------

            arm_joint_values[0] = angles::from_degrees(45.0);
            arm_joint_values[1] = angles::from_degrees(-90.0);
            arm_joint_values[2] = angles::from_degrees(0.0);
            arm_joint_values[3] = angles::from_degrees(50.0);
            arm_joint_values[4] = angles::from_degrees(0.0);
            arm_joint_values[5] = angles::from_degrees(60.0);
            arm_joint_values[6] = angles::from_degrees(0.0);

            move_group_arm.setJointValueTarget(arm_joint_values);
            move_group_arm.move();



            // --------- > Standby <-------------

            // set joint value 
            arm_joint_values[0] = angles::from_degrees(0);
            arm_joint_values[1] = angles::from_degrees(-90.0);
            arm_joint_values[2] = angles::from_degrees(0);
            arm_joint_values[3] = angles::from_degrees(110.0);
            arm_joint_values[4] = angles::from_degrees(0.0);
            arm_joint_values[5] = angles::from_degrees(-106.0);
            arm_joint_values[6] = angles::from_degrees(0.0);


            move_group_arm.setJointValueTarget(arm_joint_values);
            move_group_arm.move();


            // ===========> Place <=============
            // set joint value 
            // arm_joint_values[0] = angles::from_degrees(0);
            // arm_joint_values[1] = angles::from_degrees(-90.0);
            // arm_joint_values[2] = angles::from_degrees(0);
            // arm_joint_values[3] = angles::from_degrees(83.0);
            // arm_joint_values[4] = angles::from_degrees(0.0);
            // arm_joint_values[5] = angles::from_degrees(-71.0);
            // arm_joint_values[6] = angles::from_degrees(0.0);

            // move_group_arm.setJointValueTarget(arm_joint_values);
            // move_group_arm.move();


            move_group_arm.setPoseTarget(
                pose_presets::right_arm_downward(node->bowl_x, node->bowl_y, node->bowl_z));
            move_group_arm.move();



            // --------- > Standby <-------------

            // set joint value 
            arm_joint_values[0] = angles::from_degrees(0);
            arm_joint_values[1] = angles::from_degrees(-90.0);
            arm_joint_values[2] = angles::from_degrees(0);
            arm_joint_values[3] = angles::from_degrees(110.0);
            arm_joint_values[4] = angles::from_degrees(0.0);
            arm_joint_values[5] = angles::from_degrees(-106.0);
            arm_joint_values[6] = angles::from_degrees(0.0);

            move_group_arm.setJointValueTarget(arm_joint_values);
            move_group_arm.move();

            node->goal_status = false;
            node->motion_plan = false;

        }

        RCLCPP_INFO(rclcpp::get_logger("Result from action"),"Waiting for Action request");
        loop_rate.sleep();
    }


    RCLCPP_INFO(rclcpp::get_logger("Node"),"This node is shutting down...");
    rclcpp::shutdown();

    return 0;
}