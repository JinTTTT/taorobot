// depth_to_mm: sim-fidelity shim that makes the Gazebo depth camera look like
// the real TurtleBot 4 OAK-D driver.
//
// Gazebo's depth_camera publishes depth as 32FC1 in metres. The real depthai
// driver publishes /oakd/stereo/image_raw as 16UC1 in millimetres. This node
// subscribes to the raw gz depth (32FC1 m) and republishes 16UC1 mm on the
// public topic, so downstream deprojection code is identical on sim and robot.
//
// This node only exists to fake the hardware; it is not launched on the real
// robot.
#include <cmath>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class DepthToMm : public rclcpp::Node
{
public:
  DepthToMm()
  : Node("depth_to_mm")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "oakd/stereo/image_float", rclcpp::SensorDataQoS(),
      std::bind(&DepthToMm::on_image, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "oakd/stereo/image_raw", rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      this->get_logger(),
      "depth_to_mm: oakd/stereo/image_float (32FC1 m) -> oakd/stereo/image_raw (16UC1 mm)");
  }

private:
  void on_image(const sensor_msgs::msg::Image::SharedPtr in)
  {
    if (in->encoding != "32FC1") {
      RCLCPP_WARN_ONCE(
        this->get_logger(),
        "Expected 32FC1 depth, got '%s'; dropping frames.", in->encoding.c_str());
      return;
    }

    const size_t num_px = static_cast<size_t>(in->width) * in->height;

    sensor_msgs::msg::Image out;
    out.header = in->header;                 // keep stamp + optical frame_id
    out.height = in->height;
    out.width = in->width;
    out.encoding = "16UC1";                  // depthai convention
    out.is_bigendian = 0;
    out.step = in->width * sizeof(uint16_t);
    out.data.resize(static_cast<size_t>(out.step) * out.height);

    const float * src = reinterpret_cast<const float *>(in->data.data());
    uint16_t * dst = reinterpret_cast<uint16_t *>(out.data.data());

    for (size_t i = 0; i < num_px; ++i) {
      const float metres = src[i];
      // Invalid / no-return pixels -> 0, matching the depth-image convention.
      if (!std::isfinite(metres) || metres <= 0.0f) {
        dst[i] = 0;
        continue;
      }
      const float mm = metres * 1000.0f;
      dst[i] = (mm > 65535.0f) ? 0 : static_cast<uint16_t>(mm + 0.5f);
    }

    pub_->publish(std::move(out));
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthToMm>());
  rclcpp::shutdown();
  return 0;
}
