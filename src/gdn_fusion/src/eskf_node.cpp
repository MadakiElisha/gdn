// gdn_fusion: real-time TRN-aided error-state Kalman filter (GPS-denied nav core).
// Design record: docs/test_reports/test006..test009; open items OI-002 (baro/vertical),
// OI-003 (magnetometer yaw to replace transfer alignment).
// Frame: local NED "map" frame; map center == vehicle spawn point.
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <cmath>

namespace {
constexpr double kG = 9.80665;
constexpr double kLatC = 47.3977, kLonC = 8.5456;
constexpr double kMLat = 111320.0, kMLon = 111320.0 * 0.676876;
constexpr double kDeg = M_PI / 180.0;
constexpr int    kStaticN = 500;          // 5 s at 100 Hz
constexpr double kStillRate = 0.05;       // rad/s: exclude motion, keep vibration
constexpr double kMaxGyroMean = 0.02;     // rad/s (~1.1 deg/s): bias-plausible, motion-free
constexpr double kMaxGDev = 0.5;          // m/s^2: MEMS accel-bias-plausible
constexpr double kMaxAVar = 1.0;          // m^2/s^4 per axis: stillness variance
using V15 = Eigen::Matrix<double, 15, 1>;
using M15 = Eigen::Matrix<double, 15, 15>;
const Eigen::Vector3d kGn(0, 0, kG);

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S; S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0; return S;
}
Eigen::Vector3d clampv(const Eigen::Vector3d& v, double lim) {
    const double n = v.norm(); return (n > lim) ? v * (lim / n) : v;
}
M15 repair(const M15& P) {
    Eigen::LLT<M15> llt(P);
    if (llt.info() == Eigen::Success) return P;
    Eigen::SelfAdjointEigenSolver<M15> es(P);
    Eigen::VectorXd w = es.eigenvalues().cwiseMax(1e-9);
    return es.eigenvectors() * w.asDiagonal() * es.eigenvectors().transpose();
}
}  // namespace

class EskfNode : public rclcpp::Node {
public:
EskfNode() : Node("gdn_eskf_node") {
    map_path_  = declare_parameter("map_path", std::string("/home/madakie/gdn_workspace/data/maps/terrain_db.bin"));
    r_trn_     = declare_parameter("r_trn", 144.0);        // (12 m)^2: noise + DB mismatch
    slope_min_ = declare_parameter("slope_min", 0.02);     // observability gate
    gate_sig_  = declare_parameter("gate_sigma", 3.0);
    gate_abs_  = declare_parameter("gate_abs_m", 40.0);

    if (!LoadMap(map_path_)) { RCLCPP_ERROR(get_logger(), "map load failed: %s", map_path_.c_str()); throw std::runtime_error("map"); }
    RCLCPP_INFO(get_logger(), "DEM loaded %dx%d (smoothed, REQ-DB-001)", nx_, ny_);

    pos_ = vel_ = ba_ = bg_ = Eigen::Vector3d::Zero();
    q_ = Eigen::Quaterniond::Identity();
    x_.setZero(); P_.setZero(); Qc_.setZero();
    Qc_.block<3,3>(3,3).diagonal().setConstant(0.02 * 0.02);
    Qc_.block<3,3>(6,6).diagonal().setConstant((0.02 * kDeg) * (0.02 * kDeg));
    Qc_.block<3,3>(9,9).diagonal().setConstant(5e-4 * 5e-4);
    Qc_.block<3,3>(12,12).diagonal().setConstant((0.01 * kDeg) * (0.01 * kDeg));

    rclcpp::SensorDataQoS qos;
    imu_sub_  = create_subscription<px4_msgs::msg::SensorCombined>(
        "/fmu/out/sensor_combined", qos, std::bind(&EskfNode::ImuCb, this, std::placeholders::_1));
    lpos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos, std::bind(&EskfNode::LposCb, this, std::placeholders::_1));
    trn_sub_  = create_subscription<std_msgs::msg::Float64>(
        "/gdn/trn_meas", 10, std::bind(&EskfNode::TrnCb, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/gdn/odom", 10);
    sig_pub_  = create_publisher<std_msgs::msg::Float64>("/gdn/sigma", 10);
    sig_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
        std_msgs::msg::Float64 s; s.data = std::sqrt(P_(0,0) + P_(1,1)); sig_pub_->publish(s); });
    RCLCPP_INFO(get_logger(), "ESKF started; collecting %d static samples (hold still)", kStaticN);
}

private:
bool LoadMap(const std::string& p) {
    std::ifstream f(p, std::ios::binary); if (!f) return false;
    int32_t nx, ny;
    f.read(reinterpret_cast<char*>(&nx), 4); f.read(reinterpret_cast<char*>(&ny), 4);
    f.read(reinterpret_cast<char*>(&lat0_), 8); f.read(reinterpret_cast<char*>(&lon0_), 8);
    f.read(reinterpret_cast<char*>(&dlat_), 8); f.read(reinterpret_cast<char*>(&dlon_), 8);
    nx_ = nx; ny_ = ny; alt_.resize(size_t(nx) * ny);
    f.read(reinterpret_cast<char*>(alt_.data()), 4LL * nx * ny);
    return f.good();
}
bool MapAltGrad(double xn, double xe, double& h, double& gn, double& ge) const {
    const double fi = (kLatC + xn / kMLat - lat0_) / dlat_;
    const double fj = (kLonC + xe / kMLon - lon0_) / dlon_;
    if (fi < 1 || fi > ny_ - 2.001 || fj < 1 || fj > nx_ - 2.001) return false;
    const int i = int(fi), j = int(fj); const double ti = fi - i, tj = fj - j;
    auto A = [&](int a, int b) { return alt_[size_t(a) * nx_ + b]; };
    h  = (1-ti)*((1-tj)*A(i,j)   + tj*A(i,j+1)) + ti*((1-tj)*A(i+1,j)   + tj*A(i+1,j+1));
    gn = (A(i+1,j) - A(i-1,j)) / (2 * dlat_ * kMLat);
    ge = (A(i,j+1) - A(i,j-1)) / (2 * dlon_ * kMLon);
    return true;
}

// --- Initialisation: motion-excluding, vibration-averaging static detector.
void CollectStatic(const Eigen::Vector3d& a, const Eigen::Vector3d& w) {
    if (w.norm() < kStillRate) { n_st_++; sa_ += a; sw_ += w; sa2_ += a.cwiseAbs2(); }
    else { n_st_ = 0; sa_ = sw_ = sa2_ = Eigen::Vector3d::Zero(); }
    if (n_st_ < kStaticN) return;
    const Eigen::Vector3d ma = sa_ / n_st_, mw = sw_ / n_st_;
    const Eigen::Vector3d var = sa2_ / n_st_ - ma.cwiseAbs2();
    const double gdev = std::abs(ma.norm() - kG);
    if (mw.norm() > kMaxGyroMean || gdev > kMaxGDev || var.maxCoeff() > kMaxAVar) {
        RCLCPP_WARN(get_logger(), "static window implausible (gyro %.2f deg/s, |g| dev %.3f, aVar %.2f); recollecting",
                    mw.norm() / kDeg, gdev, var.maxCoeff());
        n_st_ = 0; sa_ = sw_ = sa2_ = Eigen::Vector3d::Zero(); return;
    }
    bg_ = mw;  // gyro bias = mean rate at rest
    const double roll  = std::atan2(-ma.y(), -ma.z());
    const double pitch = std::atan2( ma.x(), std::hypot(ma.y(), ma.z()));
    q_ = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());
    ba_ = ma - (q_.conjugate() * (-kGn));  // accel bias = residual vs leveled gravity
    P_.setZero();
    P_.block<3,3>(0,0).diagonal().setConstant(100.0);
    P_.block<3,3>(3,3).diagonal().setConstant(1.0);
    P_.block<3,3>(6,6).diagonal().setConstant((2 * kDeg) * (2 * kDeg));
    P_.block<3,3>(9,9).diagonal().setConstant(0.01);
    P_.block<3,3>(12,12).diagonal().setConstant((0.1 * kDeg) * (0.1 * kDeg));
    aligned_ = true;
    RCLCPP_INFO(get_logger(), "coarse alignment: ba=%.4f m/s2, bg=%.4f deg/s, aVar=%.3f",
                ba_.norm(), bg_.norm() / kDeg, var.maxCoeff());
}

void ImuCb(const px4_msgs::msg::SensorCombined::SharedPtr msg) {
    const Eigen::Vector3d a_m(msg->accelerometer_m_s2[0], msg->accelerometer_m_s2[1], msg->accelerometer_m_s2[2]);
    const Eigen::Vector3d w_m(msg->gyro_rad[0], msg->gyro_rad[1], msg->gyro_rad[2]);
    if (!aligned_) { CollectStatic(a_m, w_m); return; }

    const uint64_t t = msg->timestamp;
    if (last_t_ == 0) { last_t_ = t; return; }
    double dt = (t - last_t_) * 1e-6; last_t_ = t;
    if (dt <= 0) return;
    if (dt > 1.0) dt = 1.0;                       // sim-stall absorption
    if (dt > 0.1) { n_stall_++; if (n_stall_ % 10 == 1)
        RCLCPP_WARN(get_logger(), "IMU gap absorbed %.3f s (count %d)", dt, n_stall_); }

    const Eigen::Vector3d a_b = a_m - ba_, w_b = w_m - bg_;
    const int n = int(std::ceil(dt / 0.01)); const double h = dt / n;
    for (int k = 0; k < n; ++k) {                 // sub-stepped propagation
        const Eigen::Matrix3d Rm = q_.toRotationMatrix();
        M15 F = M15::Zero();
        F.block<3,3>(0,3)  = Eigen::Matrix3d::Identity();
        F.block<3,3>(3,6)  = -Rm * skew(a_b);
        F.block<3,3>(3,9)  = -Rm;
        F.block<3,3>(6,6)  = -skew(w_b);
        F.block<3,3>(6,12) = -Eigen::Matrix3d::Identity();
        const M15 Phi = M15::Identity() + F * h;
        P_ = repair(Phi * P_ * Phi.transpose() + Qc_ * h);
        const double ang = w_b.norm() * h;
        if (ang > 1e-9) q_ = (q_ * Eigen::Quaterniond(Eigen::AngleAxisd(ang, w_b.normalized()))).normalized();
        vel_ += (q_ * a_b + kGn) * h;
        pos_ += vel_ * h;
    }

    // Transfer heading alignment, once, from FC velocity while GNSS available (OI-003).
    const double spd_l = vel_lpos_.head<2>().norm();
    if (!yaw_aligned_ && have_lpos_ && spd_l > 3.0 && std::abs(w_b.z()) < 0.05) {
        const double yaw = std::atan2(vel_lpos_.y(), vel_lpos_.x());
        const Eigen::Quaterniond rz(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        const Eigen::Matrix3d Rz = rz.toRotationMatrix();
        q_ = rz * q_; pos_ = Rz * pos_; vel_ = Rz * vel_;
        x_.setZero();                      // transfer alignment = re-initialization
        P_.setZero();
        P_.block<3,3>(0,0).diagonal().setConstant(100.0);
        P_.block<3,3>(3,3).diagonal().setConstant(1.0);
        P_.block<3,3>(6,6).diagonal().setConstant((2.0 * kDeg) * (2.0 * kDeg));
        P_(8,8) += (1.0 * kDeg) * (1.0 * kDeg);   // yaw-transfer uncertainty
        P_.block<3,3>(9,9).diagonal().setConstant(0.01);
        P_.block<3,3>(12,12).diagonal().setConstant((0.1 * kDeg) * (0.1 * kDeg));
        yaw_aligned_ = true;
        RCLCPP_INFO(get_logger(), "transfer yaw %.1f deg at spd %.1f m/s; TRN on", yaw / kDeg, spd_l);
    }

    if (!yaw_aligned_ && ++n_hb_ % 1000 == 0)
        RCLCPP_INFO(get_logger(), "pre-yaw heartbeat: spd=%.1f m/s gz=%.3f have_lpos=%d",
                    spd_l, w_b.z(), int(have_lpos_));
    nav_msgs::msg::Odometry o;
    o.header.stamp = now(); o.header.frame_id = "map"; o.child_frame_id = "base_link";
    o.pose.pose.position.x = pos_.x(); o.pose.pose.position.y = pos_.y(); o.pose.pose.position.z = pos_.z();
    o.pose.pose.orientation.w = q_.w(); o.pose.pose.orientation.x = q_.x();
    o.pose.pose.orientation.y = q_.y(); o.pose.pose.orientation.z = q_.z();
    o.twist.twist.linear.x = vel_.x(); o.twist.twist.linear.y = vel_.y(); o.twist.twist.linear.z = vel_.z();
    odom_pub_->publish(o);
}

void LposCb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    vel_lpos_ = Eigen::Vector3d(msg->vx, msg->vy, msg->vz); have_lpos_ = true;
}

void TrnCb(const std_msgs::msg::Float64::SharedPtr msg) {
    if (!aligned_ || !yaw_aligned_) return;
    double h0, gn, ge;
    if (!MapAltGrad(pos_.x(), pos_.y(), h0, gn, ge)) return;
    if (std::hypot(gn, ge) < slope_min_) return;                 // observability gate
    Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
    H(0,0) = gn; H(0,1) = ge;
    const double innov = msg->data - h0 - H * x_;
    const double S = (H * P_ * H.transpose())(0,0) + r_trn_;
    const int k = n_upd_ + n_rej_;
    const double pbx = pos_.x(), pby = pos_.y();
    if (S <= 0 || std::abs(innov) / std::sqrt(S) > gate_sig_ || std::abs(innov) > gate_abs_) {
        n_rej_++;
        if (k < 40) RCLCPP_INFO(get_logger(),
            "UPD k=%d meas=%.2f h0=%.2f innov=%.2f S=%.1f acc=0 pb=(%.1f,%.1f)",
            k, msg->data, h0, innov, S, pbx, pby);
        return;
    }
    const Eigen::Matrix<double, 15, 1> K = (P_ * H.transpose()) / S;
    V15 dx = K * innov;
    dx.segment<3>(6)  = clampv(dx.segment<3>(6),  0.2 * kDeg);   // state-jump limits
    dx.segment<3>(9)  = clampv(dx.segment<3>(9),  0.005);
    dx.segment<3>(12) = clampv(dx.segment<3>(12), 0.05 * kDeg);
    x_ += dx;
    const M15 IKH = M15::Identity() - K * H;
    P_ = repair(IKH * P_ * IKH.transpose() + r_trn_ * K * K.transpose());
    pos_ += x_.segment<3>(0); vel_ += x_.segment<3>(3);
    const double da = x_.segment<3>(6).norm();
    if (da > 1e-9) q_ = (q_ * Eigen::Quaterniond(Eigen::AngleAxisd(da, x_.segment<3>(6).normalized()))).normalized();
    ba_ += x_.segment<3>(9); bg_ += x_.segment<3>(12);
    M15 J = M15::Identity(); J.block<3,3>(6,6) = Eigen::Matrix3d::Identity() - skew(x_.segment<3>(6));
    P_ = repair(J * P_ * J.transpose());
    x_.setZero();
    if (k < 40) RCLCPP_INFO(get_logger(),
        "UPD k=%d meas=%.2f h0=%.2f innov=%.2f S=%.1f acc=1 pb=(%.1f,%.1f)",
        k, msg->data, h0, innov, S, pbx, pby);
    if (++n_upd_ % 100 == 0) RCLCPP_INFO(get_logger(), "TRN updates %d (rejected %d)", n_upd_, n_rej_);
}

rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr imu_sub_;
rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr lpos_sub_;
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr trn_sub_;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sig_pub_;
rclcpp::TimerBase::SharedPtr sig_timer_;

std::string map_path_; double r_trn_, slope_min_, gate_sig_, gate_abs_;
std::vector<float> alt_; int nx_ = 0, ny_ = 0; double lat0_ = 0, lon0_ = 0, dlat_ = 1, dlon_ = 1;
M15 P_, Qc_; V15 x_;
Eigen::Vector3d pos_, vel_, ba_, bg_, sa_, sw_, sa2_, vel_lpos_;
Eigen::Quaterniond q_;
uint64_t last_t_ = 0; int n_st_ = 0, n_upd_ = 0, n_rej_ = 0, n_stall_ = 0;
bool aligned_ = false, yaw_aligned_ = false, have_lpos_ = false; int n_hb_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EskfNode>());
    rclcpp::shutdown();
    return 0;
}
