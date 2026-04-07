#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

// supported incoming poses
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using namespace sensor_msgs::msg;
using std_msgs::msg::Header;
using namespace std;

namespace pose_to_tf
{
using nav_msgs::msg::Odometry;

class AddCovariance : public rclcpp::Node
{
  rclcpp::SubscriptionBase::SharedPtr pose_sub;
  rclcpp::TimerBase::SharedPtr topic_timer;

  string topic{declare_parameter<string>("topic", "/ekf_slam/pose")};
  string parent_frame{declare_parameter<string>("parent_frame", "map")};

  double cov_xyz{declare_parameter<double>("cov.xyz", 0.01)};
  double cov_rpy{declare_parameter<double>("cov.rpy", 0.01)};
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_cov_pub;
  PoseWithCovarianceStamped pose_cov_msg;

public:
  explicit AddCovariance(rclcpp::NodeOptions options): Node("pose_with_covariance", options)
  {
    // topic to absolute in order to find it in list
    if(topic[0] != '/')
    {
      const auto ns{string(get_namespace())};
      if(ns.size() == 1)
        topic = ns + topic;
      else
        topic = ns + "/" + topic;
    }

    pose_cov_pub = create_publisher<PoseWithCovarianceStamped>(topic + "_with_cov", 1);
    pose_cov_msg.pose.covariance[0] = cov_xyz;
    pose_cov_msg.pose.covariance[7] = cov_xyz;
    pose_cov_msg.pose.covariance[14] = cov_xyz;
    pose_cov_msg.pose.covariance[21] = cov_rpy;
    pose_cov_msg.pose.covariance[28] = cov_rpy;
    pose_cov_msg.pose.covariance[35] = cov_rpy;

    topic_timer = create_wall_timer(500ms, [&](){detectTopicType();});
  }

private:


  template <class Translation=Vector3>
  inline void republish(const Quaternion &orientation, const Translation &translation = Translation())
  {
    pose_cov_msg.pose.pose.orientation = orientation;
    pose_cov_msg.pose.pose.position.x = translation.x;
    pose_cov_msg.pose.pose.position.y = translation.y;
    pose_cov_msg.pose.pose.position.z = translation.z;
    pose_cov_pub->publish(pose_cov_msg);
  }


  void detectTopicType()
  {
    const auto topics{get_topic_names_and_types()};
    const auto info{topics.find(topic)};

    if(info == topics.end())
      return;

    topic_timer.reset();

    if(info->second.size() > 1)
    {
      RCLCPP_WARN(get_logger(), topic.c_str(), "seems to have several types of message");
    }

    const auto msg{info->second[0]};

    if(msg == "geometry_msgs/msg/Pose")
    {
      pose_sub = create_subscription<Pose>(topic, 1, [&](Pose::SharedPtr msg)
                                           {
                                             pose_cov_msg.header.frame_id = parent_frame;
                                             pose_cov_msg.header.stamp = get_clock()->now();
                                             republish(msg->orientation, msg->position);
                                           });
    }
    else if(msg == "geometry_msgs/msg/PoseStamped")
    {
      pose_sub = create_subscription<PoseStamped>(topic, 1, [&](PoseStamped::SharedPtr msg)
                                                  {
                                                    pose_cov_msg.header.frame_id = parent_frame;
                                                    pose_cov_msg.header.stamp = msg->header.stamp;
                                                    republish(msg->pose.orientation, msg->pose.position);
                                                  });
    }
    else if(msg == "geometry_msgs/msg/Transform")
    {
      pose_sub = create_subscription<Transform>(topic, 1, [&](Transform::SharedPtr msg)
                                                {
                                                  pose_cov_msg.header.frame_id = parent_frame;
                                                  pose_cov_msg.header.stamp = get_clock()->now();
                                                  republish(msg->rotation, msg->translation);
                                                });
    }
    else if(msg == "geometry_msgs/msg/TransformStamped")
    {
      pose_sub = create_subscription<TransformStamped>(topic, 1, [&](TransformStamped::SharedPtr msg)
                                                       {
                                                         pose_cov_msg.header = msg->header;
                                                         republish(msg->transform.rotation, msg->transform.translation);
                                                       });
    }
    else if(msg == "nav_msgs/msg/Odometry")
    {
      pose_sub = create_subscription<Odometry>(topic, 1, [&](Odometry::SharedPtr msg)
                                               {
                                                 pose_cov_msg.header = msg->header;
                                                 republish(msg->pose.pose.orientation, msg->pose.pose.position);
                                               });
    }
    else if(msg == "sensor_msgs/msg/Imu")
    {
      pose_sub = create_subscription<Imu>(topic, rclcpp::SensorDataQoS(), [&](Imu::SharedPtr msg)
                                          {
                                            pose_cov_msg.header.frame_id = parent_frame;
                                            pose_cov_msg.header.stamp = msg->header.stamp;
                                            republish(msg->orientation);
                                          });
    }
    else
    {
      RCLCPP_ERROR(get_logger(), topic.c_str(), " has unsupported message type ", msg.c_str());
    }
  }

};
}

// boilerplate main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pose_to_tf::AddCovariance>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
