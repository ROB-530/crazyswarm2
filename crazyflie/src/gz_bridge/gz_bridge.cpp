#include <atomic>
#include <memory>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <ignition/msgs.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
 
static std::atomic<bool> g_terminatePub(false);

class Subscriber : public rclcpp::Node {
    public:
        Subscriber() : Node("gazebo_bridge") {
            mSub = create_subscription<std_msgs::msg::String>("/cf_0/iekf_pose", 10, [this](std_msgs::msg::String::ConstSharedPtr const& msg) {
                strCallback(msg);
            });
            gzPub = gzNode.Advertise<gz::msgs::StringMsg>(topic);
        }

    private:
        void strCallback(std_msgs::msg::String::ConstSharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Listen!: '%s'", msg->data.c_str());
            gzMsg.set_data(msg->data.c_str());
            gzPub.Publish(gzMsg);
        }

        gz::transport::Node gzNode;
        std::string topic = "/cf_0/iekf_pose";
        ignition::msgs::StringMsg gzMsg;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mSub;
        gz::transport::Node::Publisher gzPub;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Subscriber>());
    rclcpp::shutdown();
    return 0;
}