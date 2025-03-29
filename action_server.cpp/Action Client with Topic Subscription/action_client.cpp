#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include "action_topic_control/action/task.hpp"

using Task = action_topic_control::action::Task;
using GoalHandleTask = rclcpp_action::ClientGoalHandle<Task>;

class TaskActionClient : public rclcpp::Node {
public:
    TaskActionClient() : Node("task_action_client") {
        action_client_ = rclcpp_action::create_client<Task>(this, "execute_task");
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/task_request", 10, std::bind(&TaskActionClient::topic_callback, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Client<Task>::SharedPtr action_client_;
    std::shared_future<GoalHandleTask::SharedPtr> current_goal_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (current_goal_.valid()) {
            auto goal_handle = current_goal_.get();
            if (goal_handle) action_client_->async_cancel_goal(goal_handle);
        }
        auto goal_msg = Task::Goal();
        goal_msg.goal_description = msg->data;
        current_goal_ = action_client_->async_send_goal(goal_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskActionClient>());
    rclcpp::shutdown();
    return 0;
}
