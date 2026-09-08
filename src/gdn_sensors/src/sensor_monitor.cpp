#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

using namespace std::chrono_literals;

class SensorMonitor : public rclcpp::Node
{
public:
  SensorMonitor() : Node("gdn_sensor_monitor")
  {
    // Best-effort QoS: compatible with both reliable and best-effort publishers.
    imu_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
      "/fmu/out/sensor_combined", rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::SensorCombined::UniquePtr msg) {
        imu_count_++;
        last_accel_x_ = msg->accelerometer_m_s2[0];
      });

    lpos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
        lpos_count_++;
        last_z_ = msg->z;
      });

    timer_ = this->create_wall_timer(2s, [this]() {
      RCLCPP_INFO(this->get_logger(),
        "IMU ~%.1f Hz | LPOS ~%.1f Hz | accel_x=%.3f m/s^2 | z=%.3f m",
        static_cast<double>(imu_count_) / 2.0,
        static_cast<double>(lpos_count_) / 2.0,
        static_cast<double>(last_accel_x_),
        static_cast<double>(last_z_));
      imu_count_ = 0;
      lpos_count_ = 0;
    });
  }

private:
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr imu_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr lpos_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t imu_count_{0};
  uint64_t lpos_count_{0};
  float last_accel_x_{0.0f};
  float last_z_{0.0f};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorMonitor>());
  rclcpp::shutdown();
  return 0;
}
