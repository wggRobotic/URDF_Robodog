#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;

class MarkerPublisher : public rclcpp::Node
{
public:
    MarkerPublisher()
    : Node("marker_publisher"), counter_(0)
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "visualization_marker", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MarkerPublisher::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Marker publisher started.");
    }

private:
    void timer_callback()
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "base_link";  
        marker.header.stamp = this->now();

        marker.ns = "example_markers";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = static_cast<double>(counter_ % 5);
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.5;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        marker.lifetime = rclcpp::Duration(0, 0);  

        publisher_->publish(marker);
        RCLCPP_INFO(this->get_logger(), "Published marker at x=%.2f",
                    marker.pose.position.x);
        counter_++;
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MarkerPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
