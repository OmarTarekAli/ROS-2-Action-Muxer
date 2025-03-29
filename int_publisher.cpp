#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class IntPublisher : public rclcpp::Node {
public:
    IntPublisher() : Node("int_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/test_topic", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&IntPublisher::publish_message, this));
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_message() {
        auto msg = std_msgs::msg::Int32();
        msg.data = 42;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published Int32 message");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IntPublisher>());
    rclcpp::shutdown();
    return 0;
}
