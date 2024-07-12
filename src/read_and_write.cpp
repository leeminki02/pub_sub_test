// combine publisher and subscriber into one node

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


// The n-th node will publish to topic_n, and subscribe topics (0 ~ n-1).
class MultiNode: public rclcpp::Node {
public:
    MultiNode(int node_no): Node("multi_node"), count_ (0), node_no_(node_no) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic_"+std::to_string(node_no_), 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&MultiNode::timer_callback, this));
        for (int i = 0; i < node_no_; i++) {
            auto callback = [this, i] (const std_msgs::msg::String & msg) -> void {
                this->topic_callback(msg, i);
            };
            subscription_[i] = this->create_subscription<std_msgs::msg::String>("topic_"+std::to_string(i), 10, callback);
        }
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello #"+std::to_string(node_no_)+"! " + std::to_string(count_++);
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        printf("P: '%s'\n", message.data.c_str());
        publisher_->publish(message);
    }

    void topic_callback(const std_msgs::msg::String & msg, int topic_no) const {
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
        printf("S [#%d]: '%s'\t", topic_no, msg.data.c_str());
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    int node_no_;
    // int node_no_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_[10];

};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiNode>(std::stoi(argv[1])));
    rclcpp::shutdown();
    return 0;
}






