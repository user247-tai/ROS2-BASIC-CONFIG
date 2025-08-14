#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pkg_msgs/action/msg.hpp"

using Msg = pkg_msgs::action::Msg;
using MsgGoalHandle = rclcpp_action::ServerGoalHandle<Msg>;
using namespace std::placeholders;


class NodeServer: public rclcpp::Node
{
    public:

        NodeServer(): Node("msg_server")
        {
            // callback_gr = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            
            action_server_ = rclcpp_action::create_server<Msg>(
                this, 
                "message", 
                std::bind(&NodeServer::goal_callback, this, _1, _2),
                std::bind(&NodeServer::cancel_callback, this, _1),
                std::bind(&NodeServer::handle_accpeted_callback, this, _1),
                rcl_action_server_get_default_options(),
                // callback_gr
            );
            RCLCPP_INFO(this->get_logger(), "Action server has been started");
            
        }

        ~NodeServer(){}

    private:
        rclcpp_action::GoalResponse goal_callback(
            const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Msg::Goal> goal)
        {

            (void)uuid;
            // if (goal->data <= 1){
            //     RCLCPP_INFO(this->get_logger(), "Received a invalid goal, reject the goal");
            //     return rclcpp_action::GoalResponse::REJECT;
            // }
            RCLCPP_INFO(this->get_logger(), "Received a valid, accept and execute");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        
        rclcpp_action::CancelResponse cancel_callback(
            const std::shared_ptr<MsgGoalHandle> goal_handle)
        {
            (void)goal_handle;
            RCLCPP_INFO(this->get_logger(), "Receive a callback request, consider accept or not");
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accpeted_callback(
            const std::shared_ptr<MsgGoalHandle> goal_handle)
        {
            execute_goal(goal_handle);
        }

        void execute_goal(const std::shared_ptr<MsgGoalHandle> goal_handle)
        {

            auto result = std::make_shared<Msg::Result>();
            RCLCPP_INFO(this->get_logger(), "Executing the goal");
            
            auto data = goal_handle->get_goal()->data;

            // auto feedback = std::make_shared<Msg::Feedback>();
            
            // rclcpp::Rate looprate(1.0 / period);

            // if (!goal_handle->is_active()){
            //     result->data = data;
            //     goal_handle->abort(result);
            //     RCLCPP_INFO(this->get_logger(), "Goal has been aborted by server");
            //     return;
            // }

            // if (goal_handle->is_canceling()){
            //     result->data = data;
            //     goal_handle->canceled(result); 
            //     RCLCPP_INFO(this->get_logger(), "Goal has been cancel by server");
            //     return;
            // }

            // goal_handle->publish_feedback(feedback);


            result->data = data;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal has been executed successfully");
        }


        rclcpp_action::Server<Msg>::SharedPtr action_server_;
        // rclcpp::CallbackGroup::SharedPtr callback_gr;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}