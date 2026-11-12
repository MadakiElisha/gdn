// Thin ROS 2 wrapper around the ROS-free gdn::Eskf core + gdn::MapDb.
// All navigation math lives in include/gdn/; this file only bridges topics,
// parameters, and telemetry. Design record: docs/test_reports/.
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <variant>
#include "gdn/map_db.hpp"
#include "gdn/eskf.hpp"
#include "gdn/filter_config.hpp"
#include "gdn/ground_monitor.hpp"
#include "gdn/paths.hpp"

class EskfNode : public rclcpp::Node {
public:
    EskfNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("gdn_eskf_node", options) {
        // Relative map paths resolve against $GDN_ROOT (exported by env.sh).
        const std::string map_path = gdn::DataPath(declare_parameter("map_path", std::string(gdn::kDefaultMap)));
        // R1.10: every filter setting comes from a config file, the same one the MC loads.
        // Each key is also a parameter defaulting to the file's value, so -p KEY:=VALUE
        // overrides it (e.g. sim_run.sh passes the Gazebo world's mag_ref_*). The log names the
        // file, a fingerprint of the values run and the overrides.
        const std::string config_arg = declare_parameter("filter_config", std::string(gdn::kDefaultFilterConfig));
        gdn::FilterConfig file_cfg;
        std::string err;
        if (!gdn::LoadFilterConfig(gdn::DataPath(config_arg), &file_cfg, &err)) {
            RCLCPP_FATAL(get_logger(),
                         "filter config: %s\n  -> run `source env.sh` from the repo root, or pass "
                         "--ros-args -p filter_config:=/abs/path/to/config.yaml",
                         err.c_str());
            throw std::runtime_error("filter config load failed");
        }
        fc_ = file_cfg;
        for (const gdn::ConfigField& f : gdn::ConfigFields(&fc_)) {
            rcl_interfaces::msg::ParameterDescriptor d;
            d.dynamic_typing = true;  // so -p r_agl:=50 (an integer) can set a double
            const std::string bad = std::string("parameter ") + f.key + " has the wrong type";
            if (auto* p = std::get_if<double*>(&f.ptr)) {
                const rclcpp::ParameterValue v = declare_parameter(f.key, rclcpp::ParameterValue(**p), d);
                if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
                    **p = v.get<double>();
                else if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                    **p = static_cast<double>(v.get<int64_t>());
                else
                    throw std::runtime_error(bad);
            } else if (auto* n = std::get_if<int*>(&f.ptr)) {
                const rclcpp::ParameterValue v =
                    declare_parameter(f.key, rclcpp::ParameterValue(static_cast<int64_t>(**n)), d);
                if (v.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) throw std::runtime_error(bad);
                **n = static_cast<int>(v.get<int64_t>());
            } else {
                std::string* str = std::get<std::string*>(f.ptr);
                const rclcpp::ParameterValue v = declare_parameter(f.key, rclcpp::ParameterValue(*str), d);
                if (v.get_type() != rclcpp::ParameterType::PARAMETER_STRING) throw std::runtime_error(bad);
                *str = v.get<std::string>();
            }
        }
        if (!gdn::ValidateFilterConfig(fc_, &err)) {
            RCLCPP_FATAL(get_logger(), "filter config after parameter overrides: %s", err.c_str());
            throw std::runtime_error("bad filter config");
        }
        RCLCPP_INFO(get_logger(), "filter: %s", gdn::DescribeFilterConfig(fc_, config_arg, &file_cfg).c_str());
        const gdn::EskfConfig& cfg = fc_.eskf;
        // "agl": real altimeter model, /gdn/agl_meas + baro (R1.1, default).
        // "ideal": legacy oracle, /gdn/trn_meas is terrain height directly.
        // (Which measurement topic the node listens to, not filter tuning.)
        trn_model_ = declare_parameter("trn_model", std::string("agl"));
        // mag_fusion (R1.4, config): auto, heading, full or align; see config/filter/default.yaml.
        mag_fusion_ = fc_.mag_fusion;
        datum_sigma_ = fc_.datum_sigma_m;
        if (trn_model_ != "agl" && trn_model_ != "ideal")
            throw std::runtime_error("trn_model must be 'agl' or 'ideal'");
        if (!map_.Load(map_path)) {
            RCLCPP_FATAL(get_logger(),
                         "map load failed: %s\n  -> run `source env.sh` from the repo root, or pass "
                         "--ros-args -p map_path:=/abs/path/to/map.bin",
                         map_path.c_str());
            throw std::runtime_error("map load failed");
        }
        // Local-frame origin (R1.3). Default: the map's grid centre. When the onboard map
        // differs from the map the world or simulator uses, pass that frame's origin here so
        // both name the same place by the same coordinates (bench.sh does, for GDN_DB_MAP).
        const double o_lat = declare_parameter("origin_lat", std::numeric_limits<double>::quiet_NaN());
        const double o_lon = declare_parameter("origin_lon", std::numeric_limits<double>::quiet_NaN());
        if (std::isfinite(o_lat) && std::isfinite(o_lon)) map_.SetOrigin(o_lat, o_lon);
        RCLCPP_INFO(get_logger(), "DEM loaded %dx%d from %s (origin %.6f, %.6f), r_trn=%.0f", map_.rows(), map_.cols(),
                    map_path.c_str(), map_.lat_c(), map_.lon_c(), cfg.r_trn);
        eskf_ = std::make_unique<gdn::Eskf>(cfg);
        ground_ = std::make_unique<gdn::GroundMonitor>(fc_.ground);

        rclcpp::SensorDataQoS qos;
        imu_sub_ = create_subscription<px4_msgs::msg::SensorCombined>(
            "/fmu/out/sensor_combined", qos, std::bind(&EskfNode::ImuCb, this, std::placeholders::_1));
        lpos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos, std::bind(&EskfNode::LposCb, this, std::placeholders::_1));
        att_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos, std::bind(&EskfNode::AttCb, this, std::placeholders::_1));
        mag_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
            "/gdn/mag_meas", qos, std::bind(&EskfNode::MagCb, this, std::placeholders::_1));
        trn_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/gdn/trn_meas", 10, std::bind(&EskfNode::TrnCb, this, std::placeholders::_1));
        agl_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/gdn/agl_meas", 10, std::bind(&EskfNode::AglCb, this, std::placeholders::_1));
        baro_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/gdn/baro_meas", qos, std::bind(&EskfNode::BaroCb, this, std::placeholders::_1));
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/gdn/odom", 10);
        sig_pub_ = create_publisher<std_msgs::msg::Float64>("/gdn/sigma", 10);
        sig_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
            std_msgs::msg::Float64 m;
            m.data = eskf_->sigma_pos();
            sig_pub_->publish(m);
        });
        dbg_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
            if (!eskf_->aligned()) return;
            // ZYX Euler angles: yaw in (-180, 180], pitch in [-90, 90]. Eigen's eulerAngles()
            // keeps the first angle in [0, 180] and shows yaw 242 as "62, pitch 180, roll 180".
            const Eigen::Quaterniond& q = eskf_->q();
            const Eigen::Vector3d rpy =
                Eigen::Vector3d(
                    std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())),
                    std::asin(std::clamp(2.0 * (q.w() * q.y() - q.z() * q.x()), -1.0, 1.0)),
                    std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()))) /
                gdn::Eskf::kDeg;
            RCLCPP_INFO(get_logger(),
                        "DBG t=%.0f pos=(%.0f,%.0f,%.0f) zt=%.0f |v|=%.1f yaw=%.1f pit=%.1f rol=%.1f "
                        "ba=%.4f bg=%.4f bb=%.1f mb=(%.1f,%.1f,%.1f) sp=%.1f spz=%.1f st=%.2f pa=%.1e pe=%.1e pbz=%.1e "
                        "rej=%d(m%d b%d t%d a%d) alt=%.1f zu=%d mi=%.2f mhz=%d dp=%d",
                        t_cur_, eskf_->pos().x(), eskf_->pos().y(), eskf_->pos().z(), z_truth_, eskf_->vel().norm(),
                        rpy.x(), rpy.y(), rpy.z(), eskf_->accel_bias().norm(),
                        eskf_->gyro_bias().norm() / gdn::Eskf::kDeg, eskf_->baro_bias(), eskf_->mag_bias().x(),
                        eskf_->mag_bias().y(), eskf_->mag_bias().z(), eskf_->sigma_pos(), eskf_->sigma_pos_z(),
                        eskf_->sigma_att_deg(), eskf_->p_asym(), eskf_->p_mineig(), eskf_->p_bgz(), eskf_->rejects(),
                        eskf_->rej_mag(), eskf_->rej_baro(), eskf_->rej_trn(), eskf_->rej_agl(), eskf_->altitude_amsl(),
                        eskf_->zupt(), eskf_->mag_innov_rms(), mag_cb_, n_drop_);
            mag_cb_ = 0;
        });
        RCLCPP_INFO(get_logger(), "ESKF v3 started; collecting static samples (hold still)");
    }

private:
    void ImuCb(const px4_msgs::msg::SensorCombined::ConstSharedPtr& msg) {
        const Eigen::Vector3d a_m(msg->accelerometer_m_s2[0], msg->accelerometer_m_s2[1], msg->accelerometer_m_s2[2]);
        const Eigen::Vector3d w_m(msg->gyro_rad[0], msg->gyro_rad[1], msg->gyro_rad[2]);
        if (!eskf_->aligned()) {
            if (eskf_->FeedStatic(a_m, w_m)) {
                RCLCPP_INFO(get_logger(), "coarse alignment: ba=%.4f m/s2, bg=%.4f deg/s", eskf_->accel_bias().norm(),
                            eskf_->gyro_bias().norm() / gdn::Eskf::kDeg);
                // R1.2: on-ground detection is referenced to the aligned rest state.
                ground_->Start(eskf_->RestSpecificForce(), eskf_->gyro_bias());
            }
            return;
        }
        const uint64_t t = msg->timestamp;
        t_cur_ = static_cast<double>(t) * 1e-6;
        if (last_t_ == 0) {
            last_t_ = t;
            return;
        }
        // Signed difference: a timestamp going backwards gives dt < 0 (dropped below), not an
        // unsigned wrap to about 1.8e13 s (R1.9, clang-tidy).
        double dt = static_cast<double>(static_cast<int64_t>(t - last_t_)) * 1e-6;
        last_t_ = t;
        if (dt <= 0.0 || dt > 0.25) {
            n_drop_++;
            return;
        }  // wrap/stall: skip, never fabricate
        if (dt > 0.1) {
            n_stall_++;
            if (n_stall_ % 10 == 1) RCLCPP_WARN(get_logger(), "IMU gap absorbed %.3f s (count %d)", dt, n_stall_);
        }
        // Each sensor_combined message holds the mean rate over gyro_integral_dt. If that's
        // much shorter than the spacing between messages, samples are being dropped on the
        // way (e.g. uXRCE-DDS's 100 Hz cap with PX4 integrating at 250 Hz), and most of the
        // motion never reaches the filter. Say so loudly; the fix is IMU_INTEG_RATE.
        if (msg->gyro_integral_dt > 0 && dt * 1e6 > 1.5 * msg->gyro_integral_dt) {
            if (++n_undercover_ == 200)
                RCLCPP_ERROR(get_logger(),
                             "IMU messages cover only %.0f%% of the time between them "
                             "(integral %.1f ms, spacing %.1f ms): samples are dropped upstream. "
                             "On PX4 set IMU_INTEG_RATE to the DDS rate (100).",
                             100.0 * msg->gyro_integral_dt * 1e-6 / dt, msg->gyro_integral_dt * 1e-3, dt * 1e3);
        }
        eskf_->Propagate(a_m, w_m, dt);
        // Time not covered by this message's integral means IMU data was lost. The
        // propagation above guessed the motion in the hole from this one sample, so widen
        // the attitude and velocity uncertainty by what could have happened in it
        // (config gap_*: by default twice the current rate times the hole, plus 1 degree;
        // 2 m/s^2 for velocity).
        const double covered = msg->gyro_integral_dt > 0 ? msg->gyro_integral_dt * 1e-6 : dt;
        const double hole = dt - covered;
        if (hole > 0.02) {
            eskf_->InflateForGap(fc_.gap_att_rate_scale * (w_m - eskf_->gyro_bias()).norm() * hole +
                                     fc_.gap_att_floor_deg * gdn::Eskf::kDeg,
                                 fc_.gap_vel_accel * hole);
            if (eskf_->gaps() % 20 == 1)
                RCLCPP_WARN(get_logger(),
                            "IMU hole of %.0f ms (rate %.0f dps): attitude uncertainty widened "
                            "(count %d)",
                            hole * 1e3, w_m.norm() / gdn::Eskf::kDeg, eskf_->gaps());
        }

        // R1.2: ZUPT while the IMU says the vehicle is still; sustained motion sets the
        // airborne latch. No GNSS input (gdn/ground_monitor.hpp, TEST-018).
        const bool was_airborne = ground_->airborne();
        if (ground_->Update(a_m, w_m, dt)) eskf_->ApplyZupt();
        if (ground_->airborne() && !was_airborne)
            RCLCPP_INFO(get_logger(), "Airborne latch set at t=%.1f (IMU motion for %.1f s) - ZUPT disabled", t_cur_,
                        fc_.ground.latch_s);

        nav_msgs::msg::Odometry o;
        o.header.stamp = now();
        o.header.frame_id = "map";
        o.child_frame_id = "base_link";
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

// vehicle_local_position is PX4's GNSS-aided estimate. It is reference truth for the
// DBG line only (zt=) and is never fed to the filter (R1.2).
    void LposCb(const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr& msg) {
        if (msg->z_valid) {
            z_truth_ = msg->z;
            x_truth_ = msg->x;
            y_truth_ = msg->y;
        }
    }

    void AttCb(const px4_msgs::msg::VehicleAttitude::ConstSharedPtr& msg) {
        double qw = msg->q[0], qx = msg->q[1], qy = msg->q[2], qz = msg->q[3];
        yaw_truth_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    }

    void MagCb(const geometry_msgs::msg::Vector3::ConstSharedPtr& msg) {
        if (!eskf_->aligned()) return;
        const Eigen::Vector3d z_meas(msg->x, msg->y, msg->z);
        last_mag_meas_ = z_meas;
        mag_cb_++;
        if (!eskf_->yaw_aligned()) {
            eskf_->AlignYawFromMag(z_meas);
            RCLCPP_INFO(get_logger(), "OI-003: Instantaneous Mag Yaw Aligned at t=%.1f", t_cur_);
            return;
        }
        if (mag_fusion_ == "auto")
            eskf_->ApplyMagAuto(z_meas);
        else if (mag_fusion_ == "heading")
            eskf_->ApplyMagHeading(z_meas);
        else if (mag_fusion_ == "full")
            eskf_->ApplyMag(z_meas);
    }

    void TrnCb(const std_msgs::msg::Float64::ConstSharedPtr& msg) {
        if (trn_model_ != "ideal") return;
        const gdn::MapQuery mq = map_.Query(eskf_->pos().x(), eskf_->pos().y());
        const int k = eskf_->updates() + eskf_->rejects();
        const double pbx = eskf_->pos().x(), pby = eskf_->pos().y();
        double innov = 0.0, S = 0.0;
        const auto res = eskf_->ApplyTrn(msg->data, mq, &innov, &S);
        if (k < 40)
            RCLCPP_INFO(get_logger(), "UPD k=%d meas=%.2f h0=%.2f innov=%.2f S=%.1f acc=%d pb=(%.1f,%.1f)", k,
                        msg->data, mq.h, innov, S, res == gdn::Eskf::UpdResult::kApplied ? 1 : 0, pbx, pby);
    }

// R1.1: height above ground (tilt-corrected, metres). The first reading after yaw
// alignment fixes the altitude datum: AMSL height of p_z = 0 = map height under the
// estimate + AGL + current p_z. The baro bias is calibrated at the same moment.
    void AglCb(const std_msgs::msg::Float64::ConstSharedPtr& msg) {
        if (trn_model_ != "agl" || !eskf_->aligned() || !eskf_->yaw_aligned()) return;
        const gdn::MapQuery mq = map_.Query(eskf_->pos().x(), eskf_->pos().y());
        if (!mq.ok) return;
        if (!eskf_->datum_set()) {
            eskf_->SetAltitudeDatum(mq.h + msg->data + eskf_->pos().z(), datum_sigma_);
            if (have_baro_) eskf_->InitBaroBias(last_baro_, fc_.baro_init_sigma_m);
            z_initialized_ = true;
            RCLCPP_INFO(get_logger(), "R1.1: altitude datum %.1f m AMSL (map %.1f + AGL %.1f), baro bias %.2f m",
                        eskf_->altitude_datum(), mq.h, msg->data, eskf_->baro_bias());
            return;
        }
        // AGL alone is one equation in altitude and position: without a barometer the
        // filter can't tell "higher" from "over lower ground" (TEST-017, 67 m RMS).
        const rclcpp::Time now_t = now();
        if ((!have_baro_ || (now_t - last_baro_time_).seconds() > 2.0) &&
            (now_t - last_no_baro_warn_).seconds() > 10.0) {
            RCLCPP_WARN(get_logger(),
                        "AGL updates running without a barometer: altitude and "
                        "position are not separable; expect large horizontal error");
            last_no_baro_warn_ = now_t;
        }
        double innov = 0.0, S = 0.0;
        const int k = eskf_->agl_updates() + eskf_->rej_agl();
        const auto res = eskf_->ApplyAgl(msg->data, mq, &innov, &S);
        if (k < 20)
            RCLCPP_INFO(get_logger(), "AGL k=%d agl=%.2f innov=%.2f S=%.1f acc=%d alt=%.1f bb=%.2f", k, msg->data,
                        innov, S, res == gdn::Eskf::UpdResult::kApplied ? 1 : 0, eskf_->altitude_amsl(),
                        eskf_->baro_bias());
    }

    void BaroCb(const std_msgs::msg::Float64::ConstSharedPtr& msg) {
        // baro measurement: z_meas = z_true + bias + noise (NED)
        last_baro_ = msg->data;
        have_baro_ = true;
        last_baro_time_ = now();
        // R1.2, "ideal" mode: the local frame's origin is the start point, so the vehicle
        // starts at z = 0 and the first baro reading after alignment sets the baro bias.
        // (This used to take z from vehicle_local_position, which is GNSS-derived.) In
        // "agl" mode AglCb sets the altitude datum and the bias instead.
        if (trn_model_ == "ideal" && !z_initialized_ && eskf_->aligned()) {
            eskf_->InitBaroBias(msg->data, fc_.baro_init_sigma_m);
            z_initialized_ = true;
            RCLCPP_INFO(get_logger(), "R1.2: start altitude z=%.2f m, baro bias %.2f m from first baro",
                        eskf_->pos().z(), eskf_->baro_bias());
            return;
        }
        if (z_initialized_ && eskf_->aligned()) { eskf_->ApplyBaro(msg->data); }
    }

rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr imu_sub_;
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr baro_sub_;
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr agl_sub_;
rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr lpos_sub_;
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr trn_sub_;
rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr att_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr mag_sub_;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sig_pub_;
rclcpp::TimerBase::SharedPtr sig_timer_, dbg_timer_;

gdn::MapDb map_;
std::unique_ptr<gdn::Eskf> eskf_;
gdn::FilterConfig fc_;
std::unique_ptr<gdn::GroundMonitor> ground_;
double z_truth_ = 0.0;
    double x_truth_ = 0.0;
    double yaw_truth_ = 0.0;
    Eigen::Vector3d last_mag_meas_ = Eigen::Vector3d::Zero();
    double y_truth_ = 0.0;
uint64_t last_t_ = 0;
double t_cur_ = 0.0;
int n_stall_ = 0;
int n_undercover_ = 0;
std::string mag_fusion_ = "auto";
    int n_drop_ = 0;
    int mag_cb_ = 0;
bool z_initialized_ = false;
std::string trn_model_ = "agl";
double datum_sigma_ = gdn::kUnset;  // from the config (R1.10)
double last_baro_ = 0.0;
bool have_baro_ = false;
rclcpp::Time last_baro_time_{0, 0, RCL_ROS_TIME};
rclcpp::Time last_no_baro_warn_{0, 0, RCL_ROS_TIME};

};

// Component registration for ROS 2 composition
#include <rclcpp_components/register_node_macro.hpp>
// cppcheck-suppress unknownMacro ; defined by rclcpp_components, which cppcheck does not read
RCLCPP_COMPONENTS_REGISTER_NODE(EskfNode)

// Standalone executable entry point (only compiled into the eskf_node
// executable, not into the component library).
#ifdef GDN_STANDALONE_MAIN
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EskfNode>());
    rclcpp::shutdown();
    return 0;
}
#endif
