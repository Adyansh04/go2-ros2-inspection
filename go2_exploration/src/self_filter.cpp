// Self-filter: geometrically remove the robot's own body/legs from the 360-degree L1 cloud.
//
// Rationale: the 360-degree LiDAR sees the robot's own trunk/legs/feet. Hiding these with a large
// range_min dead-zone (0.40-0.45 m) breaks obstacle avoidance, because walls inside the dead-zone
// cannot be cleared from the costmap: with a static_layer the planner gets trapped ("No valid
// trajectories"), and without it the robot drives into walls it cannot see. Removing the body
// geometrically (a box in base_link) lets the dead-zone shrink to ~0.15 m, so walls are seen and
// cleared normally with no self-hits.
//
// Works identically in simulation and on the real Go2 (same /utlidar cloud and TF tree); the real
// Unitree L1 has the same self-occlusion problem, so this node applies to hardware as well.
//
//   in : /utlidar/cloud_deskewed   (frame: utlidar_lidar)
//   out: /utlidar/cloud_filtered   (same frame; points inside the body box dropped)
#include <cstring>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

class SelfFilter : public rclcpp::Node {
 public:
  SelfFilter() : Node("self_filter") {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    // Robot body envelope in base_link (trunk + leg/foot swing). Walls the planner navigates around
    // stay >0.42 m away (footprint 0.26 + inflation 0.16) so they fall OUTSIDE this box -> kept.
    xmin_ = declare_parameter<double>("box_x_min", -0.40);
    xmax_ = declare_parameter<double>("box_x_max", 0.40);
    ymin_ = declare_parameter<double>("box_y_min", -0.26);
    ymax_ = declare_parameter<double>("box_y_max", 0.26);
    zmin_ = declare_parameter<double>("box_z_min",
                                      -0.50);  // down to the feet/floor patch under the dog
    zmax_ = declare_parameter<double>("box_z_max", 0.40);  // up to the top of the trunk
    in_topic_ = declare_parameter<std::string>("input_topic", "/utlidar/cloud_deskewed");
    out_topic_ = declare_parameter<std::string>("output_topic", "/utlidar/cloud_filtered");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out_topic_,
                                                           rclcpp::QoS(rclcpp::KeepLast(5)));
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(in_topic_, rclcpp::SensorDataQoS(),
                                                              std::bind(&SelfFilter::cb, this, _1));
    RCLCPP_INFO(get_logger(),
                "self_filter: %s -> %s | body box x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f] in '%s'",
                in_topic_.c_str(), out_topic_.c_str(), xmin_, xmax_, ymin_, ymax_, zmin_, zmax_,
                base_frame_.c_str());
  }

 private:
  void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    tf2::Transform T;
    try {
      const auto ts =
          tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id, tf2::TimePointZero);
      T = tf2::Transform(tf2::Quaternion(ts.transform.rotation.x, ts.transform.rotation.y,
                                         ts.transform.rotation.z, ts.transform.rotation.w),
                         tf2::Vector3(ts.transform.translation.x, ts.transform.translation.y,
                                      ts.transform.translation.z));
    } catch (const std::exception& e) {
      pub_->publish(*msg);  // TF not ready yet -> pass through rather than drop the cloud
      return;
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.height = 1;
    out.fields = msg->fields;
    out.is_bigendian = msg->is_bigendian;
    out.point_step = msg->point_step;
    out.is_dense = false;
    out.data.resize(msg->data.size());

    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z");
    const uint8_t* src = msg->data.data();
    uint8_t* dst = out.data.data();
    const size_t n = static_cast<size_t>(msg->width) * msg->height;
    size_t kept = 0;
    for (size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz) {
      const tf2::Vector3 pb = T * tf2::Vector3(*ix, *iy, *iz);  // point in base_link
      const bool inside = (pb.x() >= xmin_ && pb.x() <= xmax_ && pb.y() >= ymin_ &&
                           pb.y() <= ymax_ && pb.z() >= zmin_ && pb.z() <= zmax_);
      if (!inside) {
        std::memcpy(dst + kept * out.point_step, src + i * msg->point_step, msg->point_step);
        ++kept;
      }
    }
    out.width = static_cast<uint32_t>(kept);
    out.row_step = static_cast<uint32_t>(kept) * out.point_step;
    out.data.resize(out.row_step);
    pub_->publish(out);
  }

  std::string base_frame_, in_topic_, out_topic_;
  double xmin_, xmax_, ymin_, ymax_, zmin_, zmax_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SelfFilter>());
  rclcpp::shutdown();
  return 0;
}
