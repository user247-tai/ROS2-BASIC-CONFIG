#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "bumperbot_msgs/action/move_robot.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.h"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using MoveRobot = bumperbot_msgs::action::MoveRobot;
using MoveRobotGoalHandle = rclcpp_action::ServerGoalHandle<MoveRobot>;
using Odometry = nav_msgs::msg::Odometry;
using Twist = geometry_msgs::msg::Twist;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using namespace std::placeholders;

class MoveRobotServer: public rclcpp_lifecycle::LifecycleNode{

public:
    MoveRobotServer(): LifecycleNode("move_robot_server")
    {
        this->goal_handle_ = nullptr;
        this->current_x = 0.0;
        this->current_y = 0.0;
        this->current_theta = 0.0;

        RCLCPP_INFO(this->get_logger(), "Action server has been initialized");
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "On configure...");

        callback_gr = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        
        action_server_ = rclcpp_action::create_server<MoveRobot>(
            this, 
            "move_robot", 
            std::bind(&MoveRobotServer::goal_callback, this, _1, _2),
            std::bind(&MoveRobotServer::cancel_callback, this, _1),
            std::bind(&MoveRobotServer::handle_accpeted_callback, this, _1),
            rcl_action_server_get_default_options(),
            callback_gr
        );

        this->sub_opts.callback_group = this->callback_gr;

        pose_sub_ = this->create_subscription<Odometry>(
            "odom",
            rclcpp::QoS(10), 
            std::bind(&MoveRobotServer::pose_callback, this, _1),
            this->sub_opts
        );

        vel_pub_ = this->create_publisher<Twist>(
            "cmd_vel",
            rclcpp::QoS(10),
            this->pub_opts
        );

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "On cleanup...");
        callback_gr.reset();
        action_server_.reset();
        this->sub_opts.callback_group.reset();
        pose_sub_.reset();
        vel_pub_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "On activate...");
        LifecycleNode::on_activate(previous_state);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "On deactivate...");
        LifecycleNode::on_activate(previous_state);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "On shutdown...");
        callback_gr.reset();
        action_server_.reset();
        this->sub_opts.callback_group.reset();
        pose_sub_.reset();
        vel_pub_.reset();
        return CallbackReturn::SUCCESS;
    }

    ~MoveRobotServer(){}


private:

    void pose_callback(const Odometry::SharedPtr message)
    {
        // RCLCPP_INFO(this->get_logger(), "Update robot_pose");
        this->current_x = message->pose.pose.position.x;
        this->current_y = message->pose.pose.position.y;
        geometry_msgs::msg::Quaternion q_msg = message->pose.pose.orientation;
        tf2::Quaternion q;
        tf2::fromMsg(q_msg, q);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        this->current_theta = yaw;
    }

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveRobot::Goal> goal)
    {
        (void)goal;
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received a valid, accept and execute");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<MoveRobotGoalHandle> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Receive a callback request, consider accept or not");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accpeted_callback(
        const std::shared_ptr<MoveRobotGoalHandle> goal_handle)
    {
        if ((this->goal_handle_ != nullptr) && (this->goal_handle_->is_active())){
            RCLCPP_INFO(this->get_logger(), "Received new goal, aborting current goal");
            this->aborted_goal_uuid = this->goal_handle_->get_goal_id();
        }
        execute_goal(goal_handle);
    }

    void execute_goal(const std::shared_ptr<MoveRobotGoalHandle> goal_handle)
    {

        auto result = std::make_shared<MoveRobot::Result>();
        RCLCPP_INFO(this->get_logger(), "Executing the goal");
        
        this->goal_handle_ = goal_handle;

        double goal_x = goal_handle->get_goal()->x;
        double goal_y = goal_handle->get_goal()->y;

        double distance_x = goal_x - this->current_x;
        double distance_y = goal_y - this->current_y;

        double distance = sqrt(distance_x * distance_x + distance_y * distance_y);

        auto velocity_msg = Twist();

        while(distance > 0.1){
            if (goal_handle->get_goal_id() == this->aborted_goal_uuid){
                result->message = "ABORTED";
                goal_handle->abort(result);
                RCLCPP_INFO(this->get_logger(), "Goal has been aborted by server");
                return;
            }

            if (!goal_handle->is_active()){
                result->message = "ABORTED";
                goal_handle->abort(result);
                RCLCPP_INFO(this->get_logger(), "Goal has been aborted by server");
                return;
            }

            if (goal_handle->is_canceling()){
                result->message = "CANCELED";
                goal_handle->canceled(result); 
                RCLCPP_INFO(this->get_logger(), "Goal has been cancel by server");
                return;
            }

            //Position
            velocity_msg.linear.x = 2 * distance;

            //Orientation
            float goal_theta = atan2(distance_y, distance_x);
            float diff = goal_theta - this->current_theta;
            if (diff > M_PI){
                diff -= 2*M_PI;
            }
            else if (diff < -M_PI){
                diff += 2*M_PI;
            }
            velocity_msg.angular.z = 6 * diff;
            this->vel_pub_->publish(velocity_msg);

            //Publish feedback msg
            auto feedback = std::make_shared<MoveRobot::Feedback>();
            feedback->current_x = this->current_x;
            feedback->current_y = this->current_y;
            feedback->current_theta = this->current_theta;
            RCLCPP_INFO(this->get_logger(), "Current x: %f, Current y: %f, Current z: %f", this->current_x, this->current_y, this->current_theta);
            goal_handle->publish_feedback(feedback);

            //Update distance
            distance_x = goal_x - this->current_x;
            distance_y = goal_y - this->current_y;
            distance = sqrt(distance_x * distance_x + distance_y * distance_y);
        }
        
        //Goal reached
        velocity_msg.linear.x = 0.0;
        velocity_msg.angular.z = 0.0;
        this->vel_pub_->publish(velocity_msg);

        result->message = "SUCCEEDED";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal has been executed successfully");
    }

    rclcpp_action::Server<MoveRobot>::SharedPtr action_server_;
    rclcpp::Subscription<Odometry>::SharedPtr pose_sub_;
    rclcpp::Publisher<Twist>::SharedPtr vel_pub_;
    rclcpp::PublisherOptions pub_opts;
    rclcpp::SubscriptionOptions sub_opts;
    rclcpp::CallbackGroup::SharedPtr callback_gr;
    std::shared_ptr<MoveRobotGoalHandle> goal_handle_;
    rclcpp_action::GoalUUID aborted_goal_uuid;
    double current_x;
    double current_y;
    double current_theta;

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRobotServer>();
    auto executor = rclcpp::executors::MultiThreadedExecutor();
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
