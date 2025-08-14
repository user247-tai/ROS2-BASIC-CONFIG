#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "bumperbot_msgs/action/move_until.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

using MoveUntil = bumperbot_msgs::action::MoveUntil;
using MoveUntilGoalHandle = rclcpp_action::ServerGoalHandle<MoveUntil>;
using namespace std::placeholders;


class MoveUntilServer: public rclcpp::Node
{
    public:

        MoveUntilServer(): Node("move_until_server")
        {
            callback_gr = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            
            action_server_ = rclcpp_action::create_server<MoveUntil>(
                this, 
                "move_until", 
                std::bind(&MoveUntilServer::goal_callback, this, _1, _2),
                std::bind(&MoveUntilServer::cancel_callback, this, _1),
                std::bind(&MoveUntilServer::handle_accpeted_callback, this, _1),
                rcl_action_server_get_default_options(),
                callback_gr
            );

            current_position = 50;
            this->goal_handle = nullptr;
            RCLCPP_INFO(this->get_logger(), "Action server has been started");
            
        }

        ~MoveUntilServer(){}

    private:
        rclcpp_action::GoalResponse goal_callback(
            const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveUntil::Goal> goal)
        {
            (void)uuid;
            if ((goal->position > 100) || (goal->position < 0) || ((goal->position - this->current_position) * goal->velocity) < 0){
                RCLCPP_INFO(this->get_logger(), "Received a invalid goal, reject the goal");
                return rclcpp_action::GoalResponse::REJECT;
            }
            if ((this->goal_handle != nullptr) && (this->goal_handle->is_active())){
                RCLCPP_INFO(this->get_logger(), "Accept new goal, aborting current goal");
                this->preempted_current_goal_uuid = this->goal_handle->get_goal_id();
            }
            RCLCPP_INFO(this->get_logger(), "Received a valid, accept and execute");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        
        rclcpp_action::CancelResponse cancel_callback(
            const std::shared_ptr<MoveUntilGoalHandle> goal_handle)
        {
            (void)goal_handle;
            RCLCPP_INFO(this->get_logger(), "Receive a callback request, consider accept or not");
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accpeted_callback(
            const std::shared_ptr<MoveUntilGoalHandle> goal_handle)
        {
            execute_goal(goal_handle);
        }

        void execute_goal(const std::shared_ptr<MoveUntilGoalHandle> goal_handle)
        {

            this->goal_handle = goal_handle;

            auto result = std::make_shared<MoveUntil::Result>();
            RCLCPP_INFO(this->get_logger(), "Executing the goal");
            
            auto feedback = std::make_shared<MoveUntil::Feedback>();
            
            rclcpp::Rate looprate(1.0);

            int distance_to_travel = abs(goal_handle->get_goal()->position - this->current_position);

            while (distance_to_travel != 0)
            {
                if (goal_handle->is_canceling())
                {
                    result->position = this->current_position;
                    result->message = "CANCELLED";
                    goal_handle->canceled(result); 
                    RCLCPP_INFO(this->get_logger(), "Goal has been cancel by server");
                    return;
                }
                if (goal_handle->get_goal_id() == this->preempted_current_goal_uuid){
                    RCLCPP_INFO(this->get_logger(), "Aborting current goal");
                    result->position = this->current_position;
                    result->message = "ABORTED";
                    goal_handle->abort(result);
                    return;
                }
                this->current_position += goal_handle->get_goal()->velocity;
                distance_to_travel -= abs(goal_handle->get_goal()->velocity);
                if (distance_to_travel < 0){
                    distance_to_travel = 0;
                    this->current_position = goal_handle->get_goal()->position;
                }
                feedback->current_position = this->current_position;
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Current position: %d", this->current_position);
                looprate.sleep();
            }

            // if (!goal_handle->is_active()){
            //     result->data = data;
            //     goal_handle->abort(result);
            //     RCLCPP_INFO(this->get_logger(), "Goal has been aborted by server");
            //     return;
            // }

            result->position = this->current_position;
            result->message = "SUCCEEDED";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal has been executed successfully");
        }


        rclcpp_action::Server<MoveUntil>::SharedPtr action_server_;
        rclcpp::CallbackGroup::SharedPtr callback_gr;
        std::shared_ptr<MoveUntilGoalHandle> goal_handle;
        rclcpp_action::GoalUUID preempted_current_goal_uuid;
        int current_position;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveUntilServer>();
    auto executor = rclcpp::executors::MultiThreadedExecutor();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}