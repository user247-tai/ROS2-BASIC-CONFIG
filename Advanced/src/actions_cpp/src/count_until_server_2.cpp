#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "bumperbot_msgs/action/count_until.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/callback_group.hpp"

using CountUntil = bumperbot_msgs::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ServerGoalHandle<CountUntil>;
using namespace std::placeholders;


class CountUntilServer: public rclcpp::Node
{
    public:

        CountUntilServer(): Node("count_until_client")
        {
            callback_gr = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            
            count_until_server_ = rclcpp_action::create_server<CountUntil>(
                this, 
                "count_until", 
                std::bind(&CountUntilServer::goal_callback, this, _1, _2),
                std::bind(&CountUntilServer::cancel_callback, this, _1),
                std::bind(&CountUntilServer::handle_accpeted_callback, this, _1),
                rcl_action_server_get_default_options(),
                callback_gr
            );
            RCLCPP_INFO(this->get_logger(), "Action server has been started");
            
            this->goal_handle_ = nullptr;
        }

        ~CountUntilServer(){}

    private:
        rclcpp_action::GoalResponse goal_callback(
            const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CountUntil::Goal> goal)
        {
            // if ((this->goal_handle_ != nullptr) && (this->goal_handle_->is_active())){
            //     RCLCPP_INFO(this->get_logger(), "There's a goal still active, reject goal requested");
            //     return rclcpp_action::GoalResponse::REJECT;
            // }
            (void)uuid;
            // {
            //     std::lock_guard<std::mutex> lock(mutex_);
            //     if ((this->goal_handle_ != nullptr) && (this->goal_handle_->is_active())){
            //         RCLCPP_INFO(this->get_logger(), "There's a goal active, aborting current goal and executing new goal");
            //         this->preempted_goal_uuid = this->goal_handle_->get_goal_id();
            //     }
            // }

            if (goal->target_number <= 1){
                RCLCPP_INFO(this->get_logger(), "Received a invalid goal, reject the goal");
                return rclcpp_action::GoalResponse::REJECT;
            }
            RCLCPP_INFO(this->get_logger(), "Received a valid, accept and execute");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        
        rclcpp_action::CancelResponse cancel_callback(
            const std::shared_ptr<CountUntilGoalHandle> goal_handle)
        {
            (void)goal_handle;
            RCLCPP_INFO(this->get_logger(), "Receive a callback request, consider accept or not");
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accpeted_callback(
            const std::shared_ptr<CountUntilGoalHandle> goal_handle)
        {
            if (this->goal_handle_ == nullptr){
                RCLCPP_INFO(this->get_logger(), "Accepted the goal, starting to execute the goal");
                goal_handle_queue.push(goal_handle);
                RCLCPP_INFO(this->get_logger(), "Push goal into queue, queue size: %d", this->goal_handle_queue.size());
                execute_goal(goal_handle);
            }
            else{
                RCLCPP_INFO(this->get_logger(), "Accepted the goal but there's a goal executing, push new goal into queue");
                goal_handle_queue.push(goal_handle);
                RCLCPP_INFO(this->get_logger(), "Push goal into queue, queue size: %d", this->goal_handle_queue.size());
            }

        }

        void execute_goal(const std::shared_ptr<CountUntilGoalHandle> goal_handle)
        {

            auto result = std::make_shared<CountUntil::Result>();
            RCLCPP_INFO(this->get_logger(), "Executing the goal");
            
            this->goal_handle_ = goal_handle;

            int target_number = goal_handle->get_goal()->target_number;
            double period = goal_handle->get_goal()->period;
            int counter = 0;
            auto feedback = std::make_shared<CountUntil::Feedback>();
            
            rclcpp::Rate looprate(1.0 / period);
            for (int i = 0; i < target_number; i ++){
                // {
                //     std::lock_guard<std::mutex> lock(mutex_);
                //     if (goal_handle->get_goal_id() == this->preempted_goal_uuid){
                //         result->reach_number = counter;
                //         goal_handle->abort(result);
                //         RCLCPP_INFO(this->get_logger(), "Goal has been aborted because a new goal received by server");
                //         return;
                //     }
                // }
                if (!goal_handle->is_active()){
                    result->reach_number = counter;
                    goal_handle->abort(result);
                    RCLCPP_INFO(this->get_logger(), "Goal has been aborted by server");
                    this->process_new_goal();
                    return;
                }
                if (goal_handle->is_canceling()){
                    result->reach_number = counter;
                    goal_handle->canceled(result); 
                    RCLCPP_INFO(this->get_logger(), "Goal has been cancel by server");
                    this->process_new_goal();
                    return;
                }
                goal_handle->publish_feedback(feedback);
                counter++;
                feedback->current_number = counter;
                RCLCPP_INFO(this->get_logger(), "%d", counter);
                looprate.sleep();
            }

            result->reach_number = counter;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal has been executed successfully");
            this->process_new_goal();
            RCLCPP_INFO(this->get_logger(), "Check goal in queue to process");
        }

        void process_new_goal(){
            goal_handle_queue.pop();
            RCLCPP_INFO(this->get_logger(), "Pop executed goal, queue size: %d", this->goal_handle_queue.size());
            if (goal_handle_queue.size() >= 1){
                RCLCPP_INFO(this->get_logger(), "Still have goal in queue, ready to execute new goal in queue");
                execute_goal(goal_handle_queue.front());
            }
            else{
                RCLCPP_INFO(this->get_logger(), "There's no goal in queue, reset goal_handle attribute to null");
                this->goal_handle_ = nullptr;
            }
        }

        rclcpp_action::Server<CountUntil>::SharedPtr count_until_server_;
        rclcpp::CallbackGroup::SharedPtr callback_gr;
        std::shared_ptr<CountUntilGoalHandle> goal_handle_;
        std::mutex mutex_;
        rclcpp_action::GoalUUID preempted_goal_uuid;
        std::queue<std::shared_ptr<CountUntilGoalHandle>> goal_handle_queue;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilServer>();
    auto executor = rclcpp::executors::MultiThreadedExecutor();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}