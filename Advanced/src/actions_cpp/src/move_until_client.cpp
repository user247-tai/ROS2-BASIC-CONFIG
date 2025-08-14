#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "bumperbot_msgs/action/move_until.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/qos.hpp"

using MoveUntil = bumperbot_msgs::action::MoveUntil;
using Bool = std_msgs::msg::Bool;
using namespace std::placeholders;


class MoveUntilClient: public rclcpp::Node
{

    public:
        MoveUntilClient(): Node("count_until_client"){

            this->declare_parameter("position", 80);
            this->declare_parameter("velocity", 10);

            this->position = this->get_parameter("position").as_int();
            this->velocity = this->get_parameter("velocity").as_int();

            auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

            action_client_ = rclcpp_action::create_client<MoveUntil>(
                this, 
                "move_until");

            topic_sub_ = this->create_subscription<Bool>(
                "cancel_move",
                qos,
                std::bind(&MoveUntilClient::topic_cancel_callback, this, _1));

            

            // timer_ = this->create_wall_timer(
            //                 std::chrono::seconds(5),
            //                 std::bind(&MoveUntilClient::timer_callback, this));

            rclcpp_action::ClientGoalHandle<MoveUntil>::SharedPtr goal_handle = nullptr;

        }

        void send_goal(){
            auto goal = MoveUntil::Goal();
            goal.position = this->position;
            goal.velocity = this->velocity;

            rclcpp_action::Client<MoveUntil>::SendGoalOptions options;
            options.goal_response_callback = std::bind(&MoveUntilClient::goal_response_callback, this, _1);
            options.feedback_callback = std::bind(&MoveUntilClient::goal_feedback_callback, this, _1, _2);
            options.result_callback = std::bind(&MoveUntilClient::result_callback, this, _1);  
            RCLCPP_INFO(this->get_logger(), "Starting to send goal to the server");
            action_client_->async_send_goal(goal, options);
        }

        ~MoveUntilClient(){}

    private:

        void topic_cancel_callback(Bool msg){
            if (msg.data == true){
                RCLCPP_INFO(this->get_logger(), "Cancel goal from client");
                action_client_->async_cancel_goal(this->goal_handle);
            }
        }

        void timer_callback(){
            RCLCPP_INFO(this->get_logger(), "Cancel goal from client");
            action_client_->async_cancel_goal(this->goal_handle);
            timer_->cancel();
        }

        void result_callback(const rclcpp_action::ClientGoalHandle<MoveUntil>::WrappedResult &result){
            auto status = result.code;
            if (status == rclcpp_action::ResultCode::SUCCEEDED){
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            } 
            else if (status == rclcpp_action::ResultCode::ABORTED){
                RCLCPP_INFO(this->get_logger(), "Goal aborted");
            }
            else if (status == rclcpp_action::ResultCode::CANCELED){
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
            }
            int position = result.result.get()->position;
            std::string message = result.result.get()->message;
            RCLCPP_INFO(this->get_logger(), "Received result from server: %d. Message: %s", position, message.c_str());
        }

        void goal_response_callback(const rclcpp_action::ClientGoalHandle<MoveUntil>::SharedPtr &goal_handle){
            if (!goal_handle){
                RCLCPP_INFO(this->get_logger(), "Goal rejected by server");
            }
            else{
                this->goal_handle = goal_handle;
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
            }
        }

        void goal_feedback_callback(const rclcpp_action::ClientGoalHandle<MoveUntil>::SharedPtr &goal_handle, 
                    const std::shared_ptr<const MoveUntil::Feedback> feedback){
            (void)goal_handle;
            auto data = feedback->current_position;
            RCLCPP_INFO(this->get_logger(), "Receive feedback: %d", data);
        }

        rclcpp_action::Client<MoveUntil>::SharedPtr action_client_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp_action::ClientGoalHandle<MoveUntil>::SharedPtr goal_handle;
        rclcpp::Subscription<Bool>::SharedPtr topic_sub_;
        int position;
        int velocity;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveUntilClient>();
    node->send_goal();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


