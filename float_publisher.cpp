#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class FloatPublisher : public rclcpp::Node {
public:
    FloatPublisher() : Node("float_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/test_topic", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&FloatPublisher::publish_message, this));
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_message() {
        auto msg = std_msgs::msg::Float32();
        msg.data = 3.14f;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published Float32 message");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FloatPublisher>());
    rclcpp::shutdown();
    return 0;
}
