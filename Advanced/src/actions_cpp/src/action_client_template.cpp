#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pkg_msgs/action/msg.hpp"


using Msg = pkg_msgs::action::Msg;
using namespace std::placeholders;


class NodeClient: public rclcpp::Node
{

    public:
        NodeClient(): Node("count_until_client"){
            action_client_ = rclcpp_action::create_client<Msg>(
                this, 
                "message");

            // timer_ = this->create_wall_timer(
            //                 std::chrono::seconds(5),
            //                 std::bind(&NodeClient::timer_callback, this));

            // rclcpp_action::ClientGoalHandle<Msg>::SharedPtr goal_handle = nullptr;

        }

        void send_goal(auto data){
            auto goal = Msg::Goal();
            goal.data = data;

            rclcpp_action::Client<Msg>::SendGoalOptions options;
            options.goal_response_callback = std::bind(&NodeClient::goal_response_callback, this, _1);
            options.feedback_callback = std::bind(&NodeClient::goal_feedback_callback, this, _1, _2);
            options.result_callback = std::bind(&NodeClient::result_callback, this, _1);  
            RCLCPP_INFO(this->get_logger(), "Starting to send goal to the server");
            count_until_client_->async_send_goal(goal, options);
        }

        ~NodeClient(){}

    private:

        // void timer_callback(){
        //     RCLCPP_INFO(this->get_logger(), "Cancel goal from client");
        //     action_client_->async_cancel_goal(this->goal_handle);
        //     timer_->cancel();
        // }

        void result_callback(const rclcpp_action::ClientGoalHandle<Msg>::WrappedResult &result){
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
            auto data = result.result.get()->data;
            RCLCPP_INFO(this->get_logger(), "Received result from server: %d", data);
        }

        void goal_response_callback(const rclcpp_action::ClientGoalHandle<Msg>::SharedPtr &goal_handle){
            if (!goal_handle){
                RCLCPP_INFO(this->get_logger(), "Goal rejected by server");
            }
            else{
                this->goal_handle = goal_handle;
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
            }
        }

        void goal_feedback_callback(const rclcpp_action::ClientGoalHandle<Msg>::SharedPtr &goal_handle, 
                    const std::shared_ptr<const Msg::Feedback> feedback){
            (void)goal_handle;
            auto data = feedback->data;
            RCLCPP_INFO(this->get_logger(), "Receive feedback: %d", data);
        }

        rclcpp_action::Client<Msg>::SharedPtr action_client_;
        // rclcpp::TimerBase::SharedPtr timer_;
        // rclcpp_action::ClientGoalHandle<Msg>::SharedPtr goal_handle;

};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeClient>();
    // node->send_goal(3);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



