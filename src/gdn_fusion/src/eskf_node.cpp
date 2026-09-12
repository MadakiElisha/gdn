// Thin ROS 2 wrapper around the ROS-free gdn::Eskf core + gdn::MapDb.
// All navigation math lives in include/gdn/; this file only bridges topics,
// parameters, and telemetry. Design record: docs/test_reports/.
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include <string>
#include "gdn/map_db.hpp"
#include "gdn/eskf.hpp"

class EskfNode : public rclcpp::Node {
public:
EskfNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("gdn_eskf_node", options) {
    gdn::EskfConfig cfg;
    const std::string map_path = declare_parameter("map_path",
        std::string("/home/madakie/gdn_workspace/data/maps/terrain_db_sigma4.bin"));
    cfg.r_trn      = declare_parameter("r_trn", 144.0);
    cfg.slope_min  = declare_parameter("slope_min", 0.02);
    cfg.gate_sigma = declare_parameter("gate_sigma", 3.0);
    cfg.gate_abs_m = declare_parameter("gate_abs_m", 40.0);
    cfg.clamp_dv   = declare_parameter("clamp_dv", 0.5);   // OI-006
    if (!map_.Load(map_path)) {
        RCLCPP_ERROR(get_logger(), "map load failed: %s", map_path.c_str());
        throw std::runtime_error("map load failed");
    }
    RCLCPP_INFO(get_logger(), "DEM loaded %dx%d (REQ-DB-001)", map_.rows(), map_.cols());
    eskf_ = std::make_unique<gdn::Eskf>(cfg);

    rclcpp::SensorDataQoS qos;
    imu_sub_  = create_subscription<px4_msgs::msg::SensorCombined>(
        "/fmu/out/sensor_combined", qos,
        std::bind(&EskfNode::ImuCb, this, std::placeholders::_1));
    lpos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos,
        std::bind(&EskfNode::LposCb, this, std::placeholders::_1));
    trn_sub_  = create_subscription<std_msgs::msg::Float64>(
        "/gdn/trn_meas", 10, std::bind(&EskfNode::TrnCb, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/gdn/odom", 10);
    sig_pub_  = create_publisher<std_msgs::msg::Float64>("/gdn/sigma", 10);
    sig_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
        std_msgs::msg::Float64 m; m.data = eskf_->sigma_pos(); sig_pub_->publish(m); });
    dbg_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
        if (!eskf_->aligned()) return;
        const Eigen::Vector3d rpy = eskf_->q().toRotationMatrix().eulerAngles(2, 1, 0)
                                    / gdn::Eskf::kDeg;
        RCLCPP_INFO(get_logger(),
            "DBG t=%.0f pos=(%.0f,%.0f,%.0f) |v|=%.1f yaw=%.1f pit=%.1f rol=%.1f "
            "ba=%.4f bg=%.4f sp=%.1f st=%.2f rej=%d",
            t_cur_, eskf_->pos().x(), eskf_->pos().y(), eskf_->pos().z(),
            eskf_->vel().norm(), rpy.x(), rpy.y(), rpy.z(), eskf_->accel_bias().norm(),
            eskf_->gyro_bias().norm() / gdn::Eskf::kDeg, eskf_->sigma_pos(),
            eskf_->sigma_att_deg(), eskf_->rejects());
    });
    RCLCPP_INFO(get_logger(), "ESKF v3 started; collecting static samples (hold still)");
}

private:
void ImuCb(const px4_msgs::msg::SensorCombined::SharedPtr msg) {
    const Eigen::Vector3d a_m(msg->accelerometer_m_s2[0], msg->accelerometer_m_s2[1],
                              msg->accelerometer_m_s2[2]);
    const Eigen::Vector3d w_m(msg->gyro_rad[0], msg->gyro_rad[1], msg->gyro_rad[2]);
    if (!eskf_->aligned()) {
        if (eskf_->FeedStatic(a_m, w_m))
            RCLCPP_INFO(get_logger(), "coarse alignment: ba=%.4f m/s2, bg=%.4f deg/s",
                        eskf_->accel_bias().norm(),
                        eskf_->gyro_bias().norm() / gdn::Eskf::kDeg);
        return;
    }
    const uint64_t t = msg->timestamp; t_cur_ = t * 1e-6;
    if (last_t_ == 0) { last_t_ = t; return; }
    double dt = (t - last_t_) * 1e-6; last_t_ = t;
    if (dt <= 0) return;
    if (dt > 0.1) { n_stall_++; if (n_stall_ % 10 == 1)
        RCLCPP_WARN(get_logger(), "IMU gap absorbed %.3f s (count %d)", dt, n_stall_); }
    eskf_->Propagate(a_m, w_m, dt);

    const double spd = vel_lpos_.head<2>().norm();
    if (!eskf_->yaw_aligned() && have_lpos_ && spd > 3.0 &&
        std::abs(w_m.z() - eskf_->gyro_bias().z()) < 0.05) {
        const double yaw = std::atan2(vel_lpos_.y(), vel_lpos_.x());
        eskf_->TransferAlignYaw(yaw);
        RCLCPP_INFO(get_logger(), "transfer yaw %.1f deg at spd %.1f m/s; TRN on",
                    yaw / gdn::Eskf::kDeg, spd);
    }

    nav_msgs::msg::Odometry o;
    o.header.stamp = now(); o.header.frame_id = "map"; o.child_frame_id = "base_link";
    o.pose.pose.position.x = eskf_->pos().x();
    o.pose.pose.position.y = eskf_->pos().y();
    o.pose.pose.position.z = eskf_->pos().z();
    o.pose.pose.orientation.w = eskf_->q().w();
    o.pose.pose.orientation.x = eskf_->q().x();
    o.pose.pose.orientation.y = eskf_->q().y();
    o.pose.pose.orientation.z = eskf_->q().z();
    o.twist.twist.linear.x = eskf_->vel().x();
    o.twist.twist.linear.y = eskf_->vel().y();
    o.twist.twist.linear.z = eskf_->vel().z();
    odom_pub_->publish(o);
}

void LposCb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    vel_lpos_ = Eigen::Vector3d(msg->vx, msg->vy, msg->vz); have_lpos_ = true;
}

void TrnCb(const std_msgs::msg::Float64::SharedPtr msg) {
    const gdn::MapQuery mq = map_.Query(eskf_->pos().x(), eskf_->pos().y());
    const int k = eskf_->updates() + eskf_->rejects();
    const double pbx = eskf_->pos().x(), pby = eskf_->pos().y();
    double innov = 0.0, S = 0.0;
    const auto res = eskf_->ApplyTrn(msg->data, mq, &innov, &S);
    if (k < 40)
        RCLCPP_INFO(get_logger(),
            "UPD k=%d meas=%.2f h0=%.2f innov=%.2f S=%.1f acc=%d pb=(%.1f,%.1f)",
            k, msg->data, mq.h, innov, S,
            res == gdn::Eskf::UpdResult::kApplied ? 1 : 0, pbx, pby);
}

rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr imu_sub_;
rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr lpos_sub_;
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr trn_sub_;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sig_pub_;
rclcpp::TimerBase::SharedPtr sig_timer_, dbg_timer_;

gdn::MapDb map_;
std::unique_ptr<gdn::Eskf> eskf_;
Eigen::Vector3d vel_lpos_ = Eigen::Vector3d::Zero();
uint64_t last_t_ = 0;
double t_cur_ = 0.0;
int n_stall_ = 0;
bool have_lpos_ = false;
};

// Component registration for ROS 2 composition
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(EskfNode)

// Standalone executable entry point
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EskfNode>());
    rclcpp::shutdown();
    return 0;
}
