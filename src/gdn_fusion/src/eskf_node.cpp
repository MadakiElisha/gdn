// Thin ROS 2 wrapper around the ROS-free navigation core (gdn::NavCore, R1.11).
// The decisions live in include/gdn/nav_core.hpp; this file bridges topics, parameters,
// telemetry and the run log (src/run_log.hpp). Design record: docs/test_reports/.
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <variant>
#include "gdn/map_db.hpp"
#include "gdn/eskf.hpp"
#include "gdn/filter_config.hpp"
#include "gdn/nav_core.hpp"
#include "gdn/paths.hpp"
#include "gdn_version.hpp"
#include "run_log.hpp"

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
        // "agl": real altimeter model, /gdn/agl_meas + baro (R1.1, default).
        // "ideal": legacy oracle, /gdn/trn_meas is terrain height directly.
        // (Which measurement topic the node listens to, not filter tuning.)
        const std::string trn_model = declare_parameter("trn_model", std::string("agl"));
        if (trn_model != "agl" && trn_model != "ideal") throw std::runtime_error("trn_model must be 'agl' or 'ideal'");
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
                    map_path.c_str(), map_.lat_c(), map_.lon_c(), fc_.eskf.r_trn);

        // R1.11: the run log, one MCAP per run: every input as received, every update
        // and gate decision, the state after every input, start-up events, and what is
        // needed to replay it (gdn_replay LOG). On unless log_enable is false.
        if (declare_parameter("log_enable", true)) {
            const std::string log_dir = gdn::DataPath(declare_parameter("log_dir", std::string("log/gdn_runs")));
            const std::time_t now_t = std::time(nullptr);
            char stamp[32];
            std::strftime(stamp, sizeof stamp, "%Y%m%d_%H%M%S", std::gmtime(&now_t));
            std::string dir = log_dir + "/gdn_" + stamp;
            for (int i = 2; std::filesystem::exists(dir); ++i)
                dir = log_dir + "/gdn_" + stamp + "_" + std::to_string(i);
            std::filesystem::create_directories(log_dir);
            if (!log_.Open(dir, &err)) {
                RCLCPP_FATAL(get_logger(), "run log: cannot open %s: %s (log_enable:=false to run without)",
                             dir.c_str(), err.c_str());
                throw std::runtime_error("run log open failed");
            }
            gdn_msgs::msg::RunInfo info;
            info.start_time = now();
            info.gdn_version = GDN_VERSION;
            info.config_name = fc_.name;
            info.config_path = config_arg;
            info.config_fingerprint = gdn::FilterFingerprint(fc_);
            info.config_overrides = gdn::FilterOverrides(file_cfg, fc_);
            info.config_text = gdn::DumpFilterConfig(fc_, "eskf_node run log");
            info.map_path = std::filesystem::absolute(map_path).string();
            info.map_fingerprint = gdn::FileFingerprint(map_path);
            info.origin_lat = o_lat;  // NaN: the map's centre
            info.origin_lon = o_lon;
            info.trn_model = trn_model;
            info.state_size = gdn::kStateSize;
            log_.Write(info, gdn::kTopicRunInfo, Clock());
            RCLCPP_INFO(get_logger(), "run log: %s (version %s)", dir.c_str(), GDN_VERSION);
        } else {
            RCLCPP_WARN(get_logger(), "run log off (log_enable:=false): this run can't be replayed");
        }
        sink_ = std::make_unique<gdn::RunLogSink>(&log_, [] { return Clock(); });
        sink_->on_event = [this](gdn::EventLevel level, const std::string& text, bool console) {
            if (!console) return;
            if (level == gdn::EventLevel::kError)
                RCLCPP_ERROR(get_logger(), "%s", text.c_str());
            else if (level == gdn::EventLevel::kWarn)
                RCLCPP_WARN(get_logger(), "%s", text.c_str());
            else
                RCLCPP_INFO(get_logger(), "%s", text.c_str());
        };
        sink_->on_update = [this](const gdn::Eskf::UpdateInfo& u) {
            // The first few TRN / AGL updates on the console, as before R1.11.
            const bool trn = u.sensor == gdn::Eskf::Sensor::kTrn, agl = u.sensor == gdn::Eskf::Sensor::kAgl;
            if ((!trn && !agl) || u.dim == 0 || (trn ? n_trn_print_++ >= 40 : n_agl_print_++ >= 20)) return;
            RCLCPP_INFO(get_logger(), "%s innov=%.2f S=%.1f %s alt=%.1f bb=%.2f", trn ? "UPD" : "AGL", u.y[0], u.s[0],
                        u.reason == gdn::Eskf::Reason::kApplied ? "applied" : "gated", core_->eskf().altitude_amsl(),
                        core_->eskf().baro_bias());
        };
        core_ = std::make_unique<gdn::NavCore>(
            fc_, map_, trn_model == "ideal" ? gdn::TrnModel::kIdeal : gdn::TrnModel::kAgl, sink_.get());

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
            m.data = core_->eskf().sigma_pos();
            sig_pub_->publish(m);
        });
        dbg_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
            const gdn::Eskf& e = core_->eskf();
            if (!e.aligned()) return;
            // ZYX Euler angles: yaw in (-180, 180], pitch in [-90, 90]. Eigen's eulerAngles()
            // keeps the first angle in [0, 180] and shows yaw 242 as "62, pitch 180, roll 180".
            const Eigen::Quaterniond& q = e.q();
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
                        core_->t(), e.pos().x(), e.pos().y(), e.pos().z(), z_truth_, e.vel().norm(), rpy.x(), rpy.y(),
                        rpy.z(), e.accel_bias().norm(), e.gyro_bias().norm() / gdn::Eskf::kDeg, e.baro_bias(),
                        e.mag_bias().x(), e.mag_bias().y(), e.mag_bias().z(), e.sigma_pos(), e.sigma_pos_z(),
                        e.sigma_att_deg(), e.p_asym(), e.p_mineig(), e.p_bgz(), e.rejects(), e.rej_mag(), e.rej_baro(),
                        e.rej_trn(), e.rej_agl(), e.altitude_amsl(), e.zupt(), e.mag_innov_rms(), mag_cb_,
                        core_->drops());
            mag_cb_ = 0;
        });
        RCLCPP_INFO(get_logger(), "ESKF v3 started; collecting static samples (hold still)");
    }

private:
    static std::int64_t Clock() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
            .count();
    }
    template <typename MsgT>
    void LogInput(const MsgT& msg, const char* topic) {
        log_.Write(msg, topic, Clock());
    }

    void ImuCb(const px4_msgs::msg::SensorCombined::ConstSharedPtr& msg) {
        LogInput(*msg, "/fmu/out/sensor_combined");
        const Eigen::Vector3d a_m(msg->accelerometer_m_s2[0], msg->accelerometer_m_s2[1], msg->accelerometer_m_s2[2]);
        const Eigen::Vector3d w_m(msg->gyro_rad[0], msg->gyro_rad[1], msg->gyro_rad[2]);
        if (!core_->Imu(msg->timestamp, a_m, w_m, msg->gyro_integral_dt)) return;
        const gdn::Eskf& e = core_->eskf();
        nav_msgs::msg::Odometry o;
        o.header.stamp = now();
        o.header.frame_id = "map";
        o.child_frame_id = "base_link";
        o.pose.pose.position.x = e.pos().x();
        o.pose.pose.position.y = e.pos().y();
        o.pose.pose.position.z = e.pos().z();
        o.pose.pose.orientation.w = e.q().w();
        o.pose.pose.orientation.x = e.q().x();
        o.pose.pose.orientation.y = e.q().y();
        o.pose.pose.orientation.z = e.q().z();
        o.twist.twist.linear.x = e.vel().x();
        o.twist.twist.linear.y = e.vel().y();
        o.twist.twist.linear.z = e.vel().z();
        odom_pub_->publish(o);
    }

    // vehicle_local_position is PX4's GNSS-aided estimate and vehicle_attitude PX4's
    // attitude: reference for the DBG line and the run log only, never fed to the filter
    // (R1.2).
    void LposCb(const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr& msg) {
        LogInput(*msg, "/fmu/out/vehicle_local_position");
        if (msg->z_valid) z_truth_ = msg->z;
    }

    void AttCb(const px4_msgs::msg::VehicleAttitude::ConstSharedPtr& msg) {
        LogInput(*msg, "/fmu/out/vehicle_attitude");
    }

    void MagCb(const geometry_msgs::msg::Vector3::ConstSharedPtr& msg) {
        LogInput(*msg, "/gdn/mag_meas");
        if (core_->eskf().aligned()) mag_cb_++;
        core_->Mag(Eigen::Vector3d(msg->x, msg->y, msg->z));
    }

    void TrnCb(const std_msgs::msg::Float64::ConstSharedPtr& msg) {
        LogInput(*msg, "/gdn/trn_meas");
        core_->Trn(msg->data);
    }

    void AglCb(const std_msgs::msg::Float64::ConstSharedPtr& msg) {
        LogInput(*msg, "/gdn/agl_meas");
        const gdn::Eskf& e = core_->eskf();
        const bool updating = core_->model() == gdn::TrnModel::kAgl && e.aligned() && e.yaw_aligned() && e.datum_set();
        core_->Agl(msg->data);
        // AGL alone is one equation in altitude and position: without a barometer the
        // filter can't tell "higher" from "over lower ground" (TEST-017, 67 m RMS).
        const rclcpp::Time now_t = now();
        if (updating && (!core_->have_baro() || (now_t - last_baro_time_).seconds() > 2.0) &&
            (now_t - last_no_baro_warn_).seconds() > 10.0) {
            RCLCPP_WARN(get_logger(),
                        "AGL updates running without a barometer: altitude and "
                        "position are not separable; expect large horizontal error");
            last_no_baro_warn_ = now_t;
        }
    }

    void BaroCb(const std_msgs::msg::Float64::ConstSharedPtr& msg) {
        LogInput(*msg, "/gdn/baro_meas");
        last_baro_time_ = now();
        core_->Baro(msg->data);
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
    gdn::FilterConfig fc_;
    gdn::RunLogWriter log_;  // declared before core_ and sink_: closed last
    std::unique_ptr<gdn::RunLogSink> sink_;
    std::unique_ptr<gdn::NavCore> core_;
    double z_truth_ = 0.0;
    int mag_cb_ = 0;
    int n_trn_print_ = 0, n_agl_print_ = 0;
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
