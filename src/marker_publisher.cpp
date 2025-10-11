#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using std::placeholders::_1;

class MarkerPublisher : public rclcpp::Node {
public:
  MarkerPublisher() : Node("marker_publisher") {
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization_marker_array", 10);
    gait_schedule_sub_ =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/gait_schedule", 10,
            std::bind(&MarkerPublisher::gaitScheduleCallback, this, std::placeholders::_1));
    subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/foot_contacts/mpc", 10,
        std::bind(&MarkerPublisher::callback, this, std::placeholders::_1));
    
    // Initialize gait values to default (all colorful)
    gait_values_ = {1.0f, 1.0f, 1.0f, 1.0f};
    
    RCLCPP_INFO(this->get_logger(), "Marker publisher started.");
  }

private:
  void gaitScheduleCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // Store the gait schedule values for color determination
    if (!msg->data.empty()) {
      gait_values_.clear();
      for (size_t i = 0; i < std::min(msg->data.size(), static_cast<size_t>(4)); ++i) {
        gait_values_.push_back(msg->data[i]);
      }
      // Ensure we have 4 values (pad with 0.0 if needed)
      while (gait_values_.size() < 4) {
        gait_values_.push_back(0.0f);
      }
      RCLCPP_INFO(this->get_logger(), "Updated gait schedule: [%.2f, %.2f, %.2f, %.2f]", 
                  gait_values_[0], gait_values_[1], gait_values_[2], gait_values_[3]);
    }
  }
  void callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    // Create a MarkerArray to hold all 4 foot markers
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Create 4 foot markers
    for (size_t i = 0; i < 4; ++i) {
      visualization_msgs::msg::Marker marker;
      
      marker.header.frame_id = "base_link";
      marker.header.stamp = this->now();
      
      marker.ns = "foot_contacts";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      
      // Use the pose from the PoseArray if available, otherwise use a default pose
      if (i < msg->poses.size()) {
        marker.pose = msg->poses[i];
      } else {
        // Default pose if not enough poses in the array
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
      }
      
      // Set marker scale
      marker.scale.x = 0.025;
      marker.scale.y = 0.025;
      marker.scale.z = 0.025;
      
      // Set different colors for each foot marker based on gait schedule
      if (i < gait_values_.size() && gait_values_[i] < 0.5f) {
        // Grey color when value < 0.5 (foot not in contact)
        marker.color.r = 0.5f; 
        marker.color.g = 0.5f; 
        marker.color.b = 0.5f;
      } else {
        // Colorful when value >= 0.5 (foot in contact)
        if (i == 0) {        // Front Left - Red
          marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.0f;
        } else if (i == 1) { // Front Right - Green
          marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f;
        } else if (i == 2) { // Rear Left - Blue
          marker.color.r = 0.0f; marker.color.g = 0.0f; marker.color.b = 1.0f;
        } else {             // Rear Right - Yellow
          marker.color.r = 1.0f; marker.color.g = 1.0f; marker.color.b = 0.0f;
        }
      }
      marker.color.a = 1.0f;
      
      marker.lifetime = rclcpp::Duration(0, 0); // Persistent markers
      
      // Add marker to the array
      marker_array.markers.push_back(marker);
    }
    
    // Publish all markers in a single message
    publisher_->publish(marker_array);
    
    RCLCPP_INFO(this->get_logger(), "Updated 4 foot contact markers (received %zu poses)", msg->poses.size());
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr gait_schedule_sub_;
  std::vector<float> gait_values_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MarkerPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
