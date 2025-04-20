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
#include <nav_msgs/msg/odometry.hpp>
 
static std::atomic<bool> g_terminatePub(false);

class Subscriber : public rclcpp::Node {
    public:
        Subscriber() : Node("gazebo_bridge") {
            mSub = create_subscription<nav_msgs::msg::Odometry>("/cf_0/iekf_pose", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr const& msg) {
                odomCallback(msg);
            });
            gzPub = gzNode.Advertise<gz::msgs::Odometry>(topic);
        }

    private:
        void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            auto *q = gzMsg.mutable_pose()->mutable_orientation();
            q->set_x(msg->pose.pose.orientation.x);
            q->set_y(msg->pose.pose.orientation.y);
            q->set_z(msg->pose.pose.orientation.z);
            q->set_w(msg->pose.pose.orientation.w);

            auto *p = gzMsg.mutable_pose()->mutable_position();
            p->set_x(msg->pose.pose.position.x);
            p->set_y(msg->pose.pose.position.y);
            p->set_z(msg->pose.pose.position.z);

            // ignition::msgs::Vector3d twist_p;
            // twist_p.set_x(msg->pose.twist.twist.linear.x);
            // twist_p.set_y(msg->pose.twist.twist.linear.y);
            // twist_p.set_z(msg->pose.twist.twist.linear.z);

            // ignition::msgs::Header h;
            // ignition::msgs::pose p;
            // ignition::msgs:: twist t;
            // RCLCPP_INFO(this->get_logger(), "Listen!");
            gzPub.Publish(gzMsg);
            // RCLCPP_INFO(this->get_logger(), "Done publishing!");
        }

        gz::transport::Node gzNode;
        std::string topic = "/cf_0/iekf_pose";
        ignition::msgs::Odometry gzMsg;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mSub;
        gz::transport::Node::Publisher gzPub;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Subscriber>());
    rclcpp::shutdown();
    return 0;
}