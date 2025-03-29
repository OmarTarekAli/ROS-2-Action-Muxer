#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class TaskPublisher : public rclcpp::Node {
public:
    TaskPublisher() : Node("task_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/task_request", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(2), [this]() { publish_task(); });
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_task() {
        auto msg = std_msgs::msg::String();
        msg.data = "New Task " + std::to_string(this->now().nanoseconds());
        publisher_->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskPublisher>());
    rclcpp::shutdown();
    return 0;
}
