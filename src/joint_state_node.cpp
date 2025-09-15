#include "sensor_msgs/msg/joint_state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <cmath>

class JointStateNode : public rclcpp::Node {
public:
    JointStateNode() : Node("joint_state_node") {
        joint_commands_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/joint_commands", 10,
            std::bind(&JointStateNode::jointCommandsCallback, this, std::placeholders::_1));
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);
        joint_names = {
            "FL","FL_leg","FL_foot",
            "FR","FR_leg","FR_foot",
            "BL","BL_leg","BL_foot",
            "BR","BR_leg","BR_foot"
        };
    }

private:
    void jointCommandsCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->get_clock()->now();
        js.name = joint_names;
        for (trajectory_msgs::msg::JointTrajectoryPoint point : msg->points){
            for(double position : point.positions){
                js.position.push_back(position);
                js.effort.push_back(1.0);
                js.velocity.push_back(2400/4095*M_PI_2); // ToDo: ADD servo sync
            }
        }
        joint_state_pub_->publish(js);
        return;
    }

    

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_commands_sub_;
    std::vector<std::string> joint_names;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateNode>());
    rclcpp::shutdown();
    return 0;
}
