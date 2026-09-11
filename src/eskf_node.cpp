#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <cmath>

using namespace std::chrono_literals;

class EskfNode : public rclcpp::Node {
public:
    EskfNode() : Node("gdn_eskf_node") {
        // QoS
        rclcpp::SensorDataQoS qos;

        // Subscribers
        imu_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
            "/fmu/out/sensor_combined", qos,
            std::bind(&EskfNode::imu_callback, this, std::placeholders::_1));

        // Publisher (Our fused state)
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/gdn/odom", 10);

        // Init State
        pos_ = Eigen::Vector3d::Zero();
        vel_ = Eigen::Vector3d::Zero();
        q_ = Eigen::Quaterniond::Identity();
        
        // Gravity in NED
        g_n_ << 0.0, 0.0, 9.80665;
        
        // Biases (initialized to 0, will be calibrated later)
        b_a_ = Eigen::Vector3d::Zero();
        b_g_ = Eigen::Vector3d::Zero();

        last_imu_time_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "GDN ESKF Node Started. Waiting for IMU...");
    }

private:
    void imu_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg) {
        uint64_t t = msg->timestamp;
        if (last_imu_time_ == 0) {
            last_imu_time_ = t;
            return;
        }

        double dt = (t - last_imu_time_) * 1e-6;
        last_imu_time_ = t;
        
        if (dt <= 0 || dt > 0.1) return; // Sanity check

        // Extract IMU
        Eigen::Vector3d a_m(msg->accelerometer_m_s2[0], msg->accelerometer_m_s2[1], msg->accelerometer_m_s2[2]);
        Eigen::Vector3d w_m(msg->gyro_rad[0], msg->gyro_rad[1], msg->gyro_rad[2]);

        // Correct biases
        Eigen::Vector3d a_b = a_m - b_a_;
        Eigen::Vector3d w_b = w_m - b_g_;

        // --- NOMINAL STATE PROPAGATION ---
        // 1. Attitude
        Eigen::Vector3d rot_vec = w_b * dt;
        double angle = rot_vec.norm();
        if (angle > 1e-8) {
            Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, rot_vec.normalized()));
            q_ = (q_ * dq).normalized();
        }

        // 2. Velocity
        Eigen::Vector3d a_n = q_.toRotationMatrix() * a_b + g_n_;
        vel_ += a_n * dt;

        // 3. Position
        pos_ += vel_ * dt;

        // --- PUBLISH ODOMETRY ---
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = "map"; // NED local frame
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = pos_.x();
        odom.pose.pose.position.y = pos_.y();
        odom.pose.pose.position.z = pos_.z();
        
        odom.pose.pose.orientation.w = q_.w();
        odom.pose.pose.orientation.x = q_.x();
        odom.pose.pose.orientation.y = q_.y();
        odom.pose.pose.orientation.z = q_.z();

        odom.twist.twist.linear.x = vel_.x();
        odom.twist.twist.linear.y = vel_.y();
        odom.twist.twist.linear.z = vel_.z();

        odom_pub_->publish(odom);
    }

    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    Eigen::Vector3d pos_, vel_, b_a_, b_g_, g_n_;
    Eigen::Quaterniond q_;
    uint64_t last_imu_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EskfNode>());
    rclcpp::shutdown();
    return 0;
}
