// Simulated sensors (roadmap R2.4a). Turns the GDN payload's error-free Gazebo sensors
// (bridged by tools/sim_run.sh) into what each real driver publishes: its topic,
// message type, frame, units and rate, with its errors and its timing.
//
//   ros2 run gdn_fusion sim_sensors --ros-args --params-file config/sim/sensors.yaml
//
// IMUs: interpolated onto the sensor's own clock (gdn::simsens::Resampler), corrupted
// by the R1.6 error model (gdn::ImuErrorSim, the Monte Carlo's code), acceleration in g
// where the real driver passes g through. Range (TFmini-S): the real limits and
// range-dependent noise (gdn::simsens::RangeReading), on its own 100 Hz clock from the
// latest raw beam. Every message gets a latency, a stamp (sample time plus jitter, or
// arrival time) and may be lost (gdn::simsens::TimingModel), and is published when it
// would arrive. Simulation time maps to wall time like a driver's clock
// (gdn::simsens::ClockMap). All parameters: config/sim/sensors.yaml.
#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <queue>
#include <random>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "gdn/imu_model.hpp"
#include "gdn/paths.hpp"
#include "gdn/sim_sensors.hpp"

namespace {

constexpr double kG = 9.80665;

double Sec(const builtin_interfaces::msg::Time& t) { return t.sec + t.nanosec * 1e-9; }

builtin_interfaces::msg::Time ToMsg(double s) {
    builtin_interfaces::msg::Time t;
    const double f = std::floor(s);
    t.sec = static_cast<int32_t>(f);
    t.nanosec = static_cast<uint32_t>(std::min(999999999.0, std::round((s - f) * 1e9)));
    return t;
}

// A message waiting for its arrival time.
struct Pending {
    double arrival;
    uint64_t order;
    std::function<void()> publish;
    bool operator>(const Pending& o) const { return arrival > o.arrival || (arrival == o.arrival && order > o.order); }
};

}  // namespace

class SimSensors : public rclcpp::Node {
public:
    SimSensors() : Node("gdn_sim_sensors") {
        const auto names = declare_parameter<std::vector<std::string>>("sensors", std::vector<std::string>{});
        for (const auto& n : names) {
            const std::string kind = declare_parameter<std::string>(n + ".kind");
            if (kind == "imu")
                AddImu(n);
            else if (kind == "range")
                AddRange(n);
            else
                throw std::runtime_error("sim_sensors: " + n + ": unknown kind '" + kind + "'");
        }
        timer_ = create_wall_timer(std::chrono::microseconds(500), [this] { Flush(); });
        report_ = create_wall_timer(std::chrono::seconds(10), [this] { Report(); });
    }

private:
    gdn::simsens::Timing ReadTiming(const std::string& n) {
        gdn::simsens::Timing t;
        const std::string st = declare_parameter<std::string>(n + ".stamp");
        if (st != "sensor" && st != "arrival")
            throw std::runtime_error("sim_sensors: " + n + ".stamp is sensor or arrival");
        t.arrival_stamp = st == "arrival";
        t.latency = declare_parameter<double>(n + ".latency_ms") * 1e-3;
        t.latency_jitter = declare_parameter<double>(n + ".latency_jitter_ms") * 1e-3;
        t.stamp_jitter = declare_parameter<double>(n + ".stamp_jitter_ms") * 1e-3;
        t.drop_prob = declare_parameter<double>(n + ".drop_prob");
        return t;
    }

    struct Imu {
        std::string name, frame;
        bool in_g = false;
        std::unique_ptr<gdn::simsens::Resampler> resample;
        std::unique_ptr<gdn::ImuErrorSim> err;
        std::unique_ptr<gdn::simsens::TimingModel> timing;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub;
        double t_last = -1.0;
        long n_out = 0, n_lost = 0;
    };

    void AddImu(const std::string& n) {
        auto s = std::make_unique<Imu>();
        s->name = n;
        s->frame = declare_parameter<std::string>(n + ".frame_id");
        s->in_g = declare_parameter<bool>(n + ".accel_in_g");
        const double rate = declare_parameter<double>(n + ".rate_hz");
        const double tick = declare_parameter<double>(n + ".tick_hz");
        const double skip = declare_parameter<double>(n + ".skip_prob");
        const auto seed = static_cast<uint64_t>(declare_parameter<int>(n + ".seed"));
        if (rate <= 0.0 && tick <= 0.0)
            throw std::runtime_error("sim_sensors: " + n + ": rate_hz or tick_hz must be > 0");
        gdn::ImuNoiseModel m;
        std::string err;
        const std::string path = declare_parameter<std::string>(n + ".model");
        if (!gdn::LoadImuModel(gdn::DataPath(path), &m, &err)) throw std::runtime_error("sim_sensors: " + err);
        s->resample = std::make_unique<gdn::simsens::Resampler>(rate, tick, skip, seed);
        s->err = std::make_unique<gdn::ImuErrorSim>(m, seed + 1000);
        s->timing = std::make_unique<gdn::simsens::TimingModel>(ReadTiming(n), seed + 2000);
        const std::string topic = declare_parameter<std::string>(n + ".topic");
        s->pub = create_publisher<sensor_msgs::msg::Imu>(topic, rclcpp::SensorDataQoS().keep_last(200));
        Imu* p = s.get();
        s->sub = create_subscription<sensor_msgs::msg::Imu>(
            declare_parameter<std::string>(n + ".raw_topic"), rclcpp::SensorDataQoS().keep_last(200),
            [this, p](sensor_msgs::msg::Imu::ConstSharedPtr msg) { OnImu(p, *msg); });
        RCLCPP_INFO(get_logger(), "%s: %s (%s), model %s, %s", n.c_str(), topic.c_str(), s->frame.c_str(),
                    m.name.c_str(),
                    tick > 0 ? ("tick " + std::to_string(tick) + " Hz, skip " + std::to_string(skip)).c_str()
                             : (std::to_string(rate) + " Hz").c_str());
        imus_.push_back(std::move(s));
    }

    void OnImu(Imu* s, const sensor_msgs::msg::Imu& raw) {
        const double t = Sec(raw.header.stamp);
        clock_.Observe(t, now().seconds());
        const gdn::simsens::V3 a(raw.linear_acceleration.x, raw.linear_acceleration.y, raw.linear_acceleration.z);
        const gdn::simsens::V3 w(raw.angular_velocity.x, raw.angular_velocity.y, raw.angular_velocity.z);
        s->resample->Add(t, a, w, [&](double ts, gdn::simsens::V3 aa, gdn::simsens::V3 ww) {
            const double dt = s->t_last < 0 ? s->resample->NominalDt() : ts - s->t_last;
            s->t_last = ts;
            s->err->Corrupt(&aa, &ww, dt);  // the error model runs whether or not the message is lost
            if (s->in_g) aa /= kG;
            const gdn::simsens::Message msg = s->timing->Apply(clock_.ToWall(ts));
            if (msg.lost) {
                ++s->n_lost;
                return;
            }
            auto out = std::make_shared<sensor_msgs::msg::Imu>();
            out->header.stamp = ToMsg(msg.stamp);
            out->header.frame_id = s->frame;
            out->orientation_covariance[0] = -1.0;  // no orientation (REP-145)
            out->linear_acceleration.x = aa.x();
            out->linear_acceleration.y = aa.y();
            out->linear_acceleration.z = aa.z();
            out->angular_velocity.x = ww.x();
            out->angular_velocity.y = ww.y();
            out->angular_velocity.z = ww.z();
            auto pub = s->pub;
            ++s->n_out;
            Queue(msg.arrival, [pub, out] { pub->publish(*out); });
        });
    }

    struct Range {
        std::string name, frame;
        double rate = 100.0, next = -1.0, fov = 0.0, last_raw = NAN;
        gdn::simsens::RangeSpec spec;
        std::mt19937_64 rng;
        std::unique_ptr<gdn::simsens::TimingModel> timing;
        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
        long n_out = 0, n_lost = 0;
    };

    void AddRange(const std::string& n) {
        auto s = std::make_unique<Range>();
        s->name = n;
        s->frame = declare_parameter<std::string>(n + ".frame_id");
        s->rate = declare_parameter<double>(n + ".rate_hz");
        s->spec.min_range = declare_parameter<double>(n + ".min_range");
        s->spec.max_range = declare_parameter<double>(n + ".max_range");
        s->spec.resolution = declare_parameter<double>(n + ".resolution");
        s->spec.noise_near = declare_parameter<double>(n + ".noise_near_m");
        s->spec.near_limit = declare_parameter<double>(n + ".near_limit_m");
        s->spec.noise_far_frac = declare_parameter<double>(n + ".noise_far_frac");
        s->fov = declare_parameter<double>(n + ".fov_deg") * M_PI / 180.0;
        const auto seed = static_cast<uint64_t>(declare_parameter<int>(n + ".seed"));
        s->rng.seed(seed);
        s->timing = std::make_unique<gdn::simsens::TimingModel>(ReadTiming(n), seed + 2000);
        const std::string topic = declare_parameter<std::string>(n + ".topic");
        s->pub = create_publisher<sensor_msgs::msg::Range>(topic, rclcpp::SensorDataQoS().keep_last(100));
        Range* p = s.get();
        s->sub = create_subscription<sensor_msgs::msg::LaserScan>(
            declare_parameter<std::string>(n + ".raw_topic"), rclcpp::SensorDataQoS().keep_last(100),
            [this, p](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { OnScan(p, *msg); });
        RCLCPP_INFO(get_logger(), "%s: %s (%s), %.0f Hz, %.2f to %.1f m", n.c_str(), topic.c_str(), s->frame.c_str(),
                    s->rate, s->spec.min_range, s->spec.max_range);
        ranges_.push_back(std::move(s));
    }

    // Gazebo renders the beam at its own steps (8 and 12 ms at 100 Hz on 4 ms physics);
    // the sensor reads on its own even clock, from the latest beam.
    void OnScan(Range* s, const sensor_msgs::msg::LaserScan& raw) {
        const double t = Sec(raw.header.stamp);
        clock_.Observe(t, now().seconds());
        const double r_raw = raw.ranges.empty() ? NAN : raw.ranges[0];
        if (s->next < 0) s->next = t;
        while (s->next <= t) {
            const double r_true = std::isnan(s->last_raw) ? r_raw : s->last_raw;
            Emit(s, s->next, r_true);
            s->next += 1.0 / s->rate;
        }
        s->last_raw = r_raw;
    }

    void Emit(Range* s, double ts, double r_true) {
        const double r = gdn::simsens::RangeReading(s->spec, r_true, s->rng);
        const gdn::simsens::Message msg = s->timing->Apply(clock_.ToWall(ts));
        if (msg.lost) {
            ++s->n_lost;
            return;
        }
        auto out = std::make_shared<sensor_msgs::msg::Range>();
        out->header.stamp = ToMsg(msg.stamp);
        out->header.frame_id = s->frame;
        out->radiation_type = sensor_msgs::msg::Range::INFRARED;
        out->field_of_view = static_cast<float>(s->fov);
        out->min_range = static_cast<float>(s->spec.min_range);
        out->max_range = static_cast<float>(s->spec.max_range);
        out->range = static_cast<float>(r);
        auto pub = s->pub;
        ++s->n_out;
        Queue(msg.arrival, [pub, out] { pub->publish(*out); });
    }

    void Queue(double arrival, std::function<void()> f) { q_.push(Pending{arrival, order_++, std::move(f)}); }

    void Flush() {
        const double now_s = now().seconds();
        while (!q_.empty() && q_.top().arrival <= now_s) {
            q_.top().publish();
            q_.pop();
        }
    }

    void Report() {
        std::string msg = "over 10 s:";
        for (auto& s : imus_) {
            msg += " " + s->name + " " + std::to_string(s->n_out / 10) + " Hz (" + std::to_string(s->n_lost) + " lost)";
            s->n_out = s->n_lost = 0;
        }
        for (auto& s : ranges_) {
            msg += " " + s->name + " " + std::to_string(s->n_out / 10) + " Hz (" + std::to_string(s->n_lost) + " lost)";
            s->n_out = s->n_lost = 0;
        }
        if (!clock_.Ready()) msg += "; no Gazebo sensor data yet (is the bridge running?)";
        RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    }

    gdn::simsens::ClockMap clock_;
    std::vector<std::unique_ptr<Imu>> imus_;
    std::vector<std::unique_ptr<Range>> ranges_;
    std::priority_queue<Pending, std::vector<Pending>, std::greater<Pending>> q_;
    uint64_t order_ = 0;
    rclcpp::TimerBase::SharedPtr timer_, report_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimSensors>());
    rclcpp::shutdown();
    return 0;
}
