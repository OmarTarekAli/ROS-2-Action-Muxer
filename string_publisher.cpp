#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class StringPublisher : public rclcpp::Node {
public:
    StringPublisher() : Node("string_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&StringPublisher::publish_message, this));
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_message() {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello from StringPublisher";
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published String message");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StringPublisher>());
    rclcpp::shutdown();
    return 0;
}
