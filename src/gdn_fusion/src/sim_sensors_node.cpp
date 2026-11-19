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
// would arrive. All of that runs on Gazebo's clock (gdn::simsens::SimClock): stamps are
// Gazebo time plus one offset fixed at start-up (logged: "clock: stamps are Gazebo time
// + ... s"), so their spacing is the simulation's even when it runs slower than real
// time. All parameters: config/sim/sensors.yaml.
//
// Stereo (R2.4b, the OAK-D Lite's mono pair): Gazebo renders both cameras at the same
// simulation time; the pair is matched on that time and gets one timing draw, so both
// images carry the same stamp and are lost together, as the real hardware-synced pair.
// Gazebo's grey cameras give linear light (no gamma), so the pair is rendered at 16 bits
// and made into what a camera sends: one auto-exposure gain for both
// (gdn::simsens::AutoExposure), sRGB-encoded to 8 bits (gdn::simsens::ToneMap), mono8.
// Each image gets read noise (gdn::simsens::ImageNoise) and its camera_info: the
// distortion-free pinhole of gdn::simsens::PinholeFromHfov, and for the right camera
// the baseline in its projection (Tx = -fx * baseline, as a rectified ROS pair).
//
// Lidar (R2.4c, the Mid-360S, built when livox_ros_driver2's messages are found): Gazebo
// renders a dense error-free grid of rays (gdn::simsens::LidarGrid) several times a
// frame; each point of Livox's measured pattern (gdn::simsens::LivoxPattern) keeps its
// own direction, its range from the four rays around it (gdn::simsens::GridRange), each
// interpolated between the renders around the point's own time
// (gdn::simsens::InterpRange), and fires on the real stream's packet clock
// (gdn::simsens::LivoxTiming). Points with no return are (0, 0, 0), reflectivity 0, as
// the driver publishes them. Each 100 ms frame goes out as livox_ros_driver2/CustomMsg:
// header stamp = timebase = the first point's time, offset_time per point, line 0 to 3.
//
// Every publisher is reliable, as the real drivers' are (tools/h30wp_node.py, depthai-
// ros, livox_ros_driver2): a reliable publisher also serves best-effort subscribers,
// but not the other way round, and VIO nodes subscribe reliable.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <initializer_list>
#include <map>
#include <memory>
#include <queue>
#include <random>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#ifdef GDN_HAVE_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

#include "gdn/imu_model.hpp"
#include "gdn/paths.hpp"
#include "gdn/sim_sensors.hpp"

namespace {

constexpr double kG = 9.80665;

double Sec(const builtin_interfaces::msg::Time& t) { return t.sec + t.nanosec * 1e-9; }

// "sim_sensors: " followed by the parts, for a configuration error.
[[noreturn]] void ConfigError(std::initializer_list<std::string> parts) {
    std::string msg = "sim_sensors: ";
    for (const auto& s : parts) msg += s;
    throw std::runtime_error(msg);
}

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
            else if (kind == "stereo")
                AddStereo(n);
            else if (kind == "livox")
                AddLivox(n);
            else
                ConfigError({n, ": unknown kind '", kind, "'"});
        }
        timer_ = create_wall_timer(std::chrono::microseconds(500), [this] { Flush(); });
        report_ = create_wall_timer(std::chrono::seconds(10), [this] { Report(); });
    }

private:
    gdn::simsens::Timing ReadTiming(const std::string& n) {
        gdn::simsens::Timing t;
        const std::string st = declare_parameter<std::string>(n + ".stamp");
        if (st != "sensor" && st != "arrival") ConfigError({n, ".stamp is sensor or arrival"});
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
        if (rate <= 0.0 && tick <= 0.0) ConfigError({n, ": rate_hz or tick_hz must be > 0"});
        gdn::ImuNoiseModel m;
        std::string err;
        const std::string path = declare_parameter<std::string>(n + ".model");
        if (!gdn::LoadImuModel(gdn::DataPath(path), &m, &err)) ConfigError({err});
        s->resample = std::make_unique<gdn::simsens::Resampler>(rate, tick, skip, seed);
        s->err = std::make_unique<gdn::ImuErrorSim>(m, seed + 1000);
        s->timing = std::make_unique<gdn::simsens::TimingModel>(ReadTiming(n), seed + 2000);
        const std::string topic = declare_parameter<std::string>(n + ".topic");
        s->pub = create_publisher<sensor_msgs::msg::Imu>(topic, rclcpp::QoS(200));
        Imu* p = s.get();
        s->sub = create_subscription<sensor_msgs::msg::Imu>(
            declare_parameter<std::string>(n + ".raw_topic"), rclcpp::SensorDataQoS().keep_last(200),
            [this, p](const sensor_msgs::msg::Imu::ConstSharedPtr& msg) { OnImu(p, *msg); });
        RCLCPP_INFO(get_logger(), "%s: %s (%s), model %s, %s", n.c_str(), topic.c_str(), s->frame.c_str(),
                    m.name.c_str(),
                    tick > 0 ? ("tick " + std::to_string(tick) + " Hz, skip " + std::to_string(skip)).c_str()
                             : (std::to_string(rate) + " Hz").c_str());
        imus_.push_back(std::move(s));
    }

    void OnImu(Imu* s, const sensor_msgs::msg::Imu& raw) {
        const double t = Sec(raw.header.stamp);
        ObserveClock(t);
        const gdn::simsens::V3 a(raw.linear_acceleration.x, raw.linear_acceleration.y, raw.linear_acceleration.z);
        const gdn::simsens::V3 w(raw.angular_velocity.x, raw.angular_velocity.y, raw.angular_velocity.z);
        s->resample->Add(t, a, w, [&](double ts, gdn::simsens::V3 aa, gdn::simsens::V3 ww) {
            const double dt = s->t_last < 0 ? s->resample->NominalDt() : ts - s->t_last;
            s->t_last = ts;
            s->err->Corrupt(&aa, &ww, dt);  // the error model runs whether or not the message is lost
            if (s->in_g) aa /= kG;
            // Not before the raw sample at t, which this one is interpolated towards.
            const gdn::simsens::Message msg = s->timing->Apply(ts, t);
            if (msg.lost) {
                ++s->n_lost;
                return;
            }
            auto out = std::make_shared<sensor_msgs::msg::Imu>();
            out->header.stamp = ToMsg(clock_.ToStamp(msg.stamp));
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
        double rate = 100.0, next = -1.0, start = 0.0, fov = 0.0, last_raw = NAN;
        int64_t n = 0;  // readings since start (an integer count: no rounding build-up)
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
        s->pub = create_publisher<sensor_msgs::msg::Range>(topic, rclcpp::QoS(100));
        Range* p = s.get();
        s->sub = create_subscription<sensor_msgs::msg::LaserScan>(
            declare_parameter<std::string>(n + ".raw_topic"), rclcpp::SensorDataQoS().keep_last(100),
            [this, p](const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) { OnScan(p, *msg); });
        RCLCPP_INFO(get_logger(), "%s: %s (%s), %.0f Hz, %.2f to %.1f m", n.c_str(), topic.c_str(), s->frame.c_str(),
                    s->rate, s->spec.min_range, s->spec.max_range);
        ranges_.push_back(std::move(s));
    }

    // Gazebo renders the beam at its own steps (8 and 12 ms at 100 Hz on 4 ms physics);
    // the sensor reads on its own even clock, from the latest beam, as Gazebo's time
    // passes each reading (R2.4b: R2.4a waited for the next beam, up to 12 ms late).
    void OnScan(Range* s, const sensor_msgs::msg::LaserScan& raw) {
        const double t = Sec(raw.header.stamp);
        ObserveClock(t);
        if (s->next < 0) s->start = s->next = t;
        TickRange(s, t);  // readings before this beam read the one before
        s->last_raw = raw.ranges.empty() ? NAN : raw.ranges[0];
    }

    void TickRange(Range* s, double now_sim) {
        if (s->next < 0 || std::isnan(s->last_raw)) return;
        while (s->next <= now_sim) {
            Emit(s, s->next, s->last_raw);
            s->next = s->start + static_cast<double>(++s->n) / s->rate;
        }
    }

    void Emit(Range* s, double ts, double r_true) {
        const double r = gdn::simsens::RangeReading(s->spec, r_true, s->rng);
        const gdn::simsens::Message msg = s->timing->Apply(ts);
        if (msg.lost) {
            ++s->n_lost;
            return;
        }
        auto out = std::make_shared<sensor_msgs::msg::Range>();
        out->header.stamp = ToMsg(clock_.ToStamp(msg.stamp));
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

    struct Camera {
        std::string frame;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
        sensor_msgs::msg::CameraInfo info;
        std::map<double, sensor_msgs::msg::Image::ConstSharedPtr> waiting;  // by Gazebo time
    };

    struct Stereo {
        std::string name;
        int width = 0, height = 0;
        Camera cam[2];  // left, right
        std::unique_ptr<gdn::simsens::AutoExposure> ae;
        gdn::simsens::ToneMap tone;
        double t_last = -1.0;
        std::unique_ptr<gdn::simsens::ImageNoise> noise;
        std::unique_ptr<gdn::simsens::TimingModel> timing;
        long n_out = 0, n_lost = 0, n_unpaired = 0;
    };

    void AddStereo(const std::string& n) {
        auto s = std::make_unique<Stereo>();
        s->name = n;
        s->width = static_cast<int>(declare_parameter<int>(n + ".width"));
        s->height = static_cast<int>(declare_parameter<int>(n + ".height"));
        const double hfov = declare_parameter<double>(n + ".hfov_deg") * M_PI / 180.0;
        const double baseline = declare_parameter<double>(n + ".baseline_m");
        const auto seed = static_cast<uint64_t>(declare_parameter<int>(n + ".seed"));
        if (s->width <= 0 || s->height <= 0 || hfov <= 0.0 || hfov >= M_PI || baseline <= 0.0)
            ConfigError({n, ": width, height, hfov_deg (0 to 180) and baseline_m must be > 0"});
        const gdn::simsens::Pinhole k = gdn::simsens::PinholeFromHfov(s->width, s->height, hfov);
        s->noise = std::make_unique<gdn::simsens::ImageNoise>(declare_parameter<double>(n + ".noise_dn"), seed);
        const auto ae = declare_parameter<std::vector<double>>(n + ".auto_exposure");  // target, tau, gain range
        if (ae.size() != 4 || ae[0] <= 0.0 || ae[1] <= 0.0 || ae[2] <= 0.0 || ae[3] < ae[2])
            ConfigError({n, ".auto_exposure is [target, tau_s, gain_min, gain_max]"});
        s->ae = std::make_unique<gdn::simsens::AutoExposure>(ae[0], ae[1], ae[2], ae[3]);
        s->timing = std::make_unique<gdn::simsens::TimingModel>(ReadTiming(n), seed + 2000);
        const char* side[2] = {"left", "right"};
        Stereo* p = s.get();
        for (int i = 0; i < 2; ++i) {
            Camera& c = s->cam[i];
            const std::string pre = n + "." + side[i] + "_";
            c.frame = declare_parameter<std::string>(pre + "frame_id");
            auto& info = c.info;
            info.header.frame_id = c.frame;
            info.width = static_cast<uint32_t>(s->width);
            info.height = static_cast<uint32_t>(s->height);
            info.distortion_model = "plumb_bob";
            info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
            info.k = {k.fx, 0.0, k.cx, 0.0, k.fy, k.cy, 0.0, 0.0, 1.0};
            info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
            info.p = {k.fx, 0.0, k.cx, i == 1 ? -k.fx * baseline : 0.0, 0.0, k.fy, k.cy, 0.0, 0.0, 0.0, 1.0, 0.0};
            const std::string topic = declare_parameter<std::string>(pre + "topic");
            c.pub = create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(5));
            c.info_pub = create_publisher<sensor_msgs::msg::CameraInfo>(
                declare_parameter<std::string>(pre + "info_topic"), rclcpp::QoS(5));
            c.sub = create_subscription<sensor_msgs::msg::Image>(
                declare_parameter<std::string>(pre + "raw_topic"), rclcpp::QoS(5),
                [this, p, i](const sensor_msgs::msg::Image::ConstSharedPtr& msg) { OnImage(p, i, msg); });
            RCLCPP_INFO(get_logger(), "%s: %s (%s)", n.c_str(), topic.c_str(), c.frame.c_str());
        }
        RCLCPP_INFO(get_logger(), "%s: %dx%d, fx %.2f, cx %.1f, cy %.1f, baseline %.3f m", n.c_str(), s->width,
                    s->height, k.fx, k.cx, k.cy, baseline);
        stereos_.push_back(std::move(s));
    }

    // Holds each image until its partner from the same Gazebo time arrives; an image
    // whose partner never comes is dropped once a newer pair is out. The raw images are
    // subscribed reliable: best-effort lost a few of these 600 kB messages every 10 s.
    void OnImage(Stereo* s, int side, const sensor_msgs::msg::Image::ConstSharedPtr& raw) {
        const double t = Sec(raw->header.stamp);
        ObserveClock(t);
        if (raw->encoding != "mono16" || raw->is_bigendian || static_cast<int>(raw->width) != s->width ||
            static_cast<int>(raw->height) != s->height || raw->data.size() < size_t{2} * raw->width * raw->height) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s: raw image is %s %ux%u, want mono16 %dx%d",
                                  s->name.c_str(), raw->encoding.c_str(), raw->width, raw->height, s->width, s->height);
            return;
        }
        auto& mine = s->cam[side].waiting;
        mine[t] = raw;
        if (mine.size() > 10) {  // its partners stopped coming
            mine.erase(mine.begin());
            ++s->n_unpaired;
        }
        auto& other = s->cam[1 - side].waiting;
        const auto it = other.find(t);
        if (it == other.end()) return;
        const auto pair = side == 0 ? std::make_pair(raw, it->second) : std::make_pair(it->second, raw);
        for (auto& c : s->cam) {
            const auto stale = c.waiting.upper_bound(t);
            for (auto j = c.waiting.begin(); j != stale; ++j)
                if (j->first < t) ++s->n_unpaired;
            c.waiting.erase(c.waiting.begin(), stale);
        }
        EmitPair(s, t, pair.first, pair.second);
    }

    void EmitPair(Stereo* s, double t, const sensor_msgs::msg::Image::ConstSharedPtr& left,
                  const sensor_msgs::msg::Image::ConstSharedPtr& right) {
        const gdn::simsens::Message msg = s->timing->Apply(t);
        if (msg.lost) {
            ++s->n_lost;
            return;
        }
        const sensor_msgs::msg::Image::ConstSharedPtr raw[2] = {left, right};
        const size_t npx = static_cast<size_t>(s->width) * static_cast<size_t>(s->height);
        // One exposure for the pair, from both images' mean (every 7th pixel).
        double sum = 0.0;
        size_t cnt = 0;
        for (const auto& r : raw) {
            const auto* px = reinterpret_cast<const uint16_t*>(r->data.data());
            for (size_t j = 0; j < npx; j += 7, ++cnt) sum += px[j];
        }
        s->tone.SetGain(s->ae->Update(sum / (65535.0 * static_cast<double>(cnt)), s->t_last < 0 ? 0.0 : t - s->t_last));
        s->t_last = t;
        for (int i = 0; i < 2; ++i) {
            Camera& c = s->cam[i];
            auto img = std::make_shared<sensor_msgs::msg::Image>();
            img->header.stamp = ToMsg(clock_.ToStamp(msg.stamp));
            img->header.frame_id = c.frame;
            img->width = static_cast<uint32_t>(s->width);
            img->height = static_cast<uint32_t>(s->height);
            img->encoding = "mono8";
            img->step = img->width;
            img->data.resize(npx);
            s->tone.Apply(reinterpret_cast<const uint16_t*>(raw[i]->data.data()), img->data.data(), npx);
            s->noise->Apply(img->data.data(), img->data.size());
            auto info = std::make_shared<sensor_msgs::msg::CameraInfo>(c.info);
            info->header.stamp = img->header.stamp;
            auto pub = c.pub;
            auto info_pub = c.info_pub;
            Queue(msg.arrival, [pub, info_pub, img, info] {
                pub->publish(*img);
                info_pub->publish(*info);
            });
        }
        ++s->n_out;
    }

#ifdef GDN_HAVE_LIVOX
    struct Lidar {
        std::string name, frame;
        uint8_t lidar_id = 192, reflectivity = 40;
        gdn::simsens::LivoxPattern pattern;
        gdn::simsens::LidarGrid grid;
        gdn::simsens::LivoxTiming tim;
        gdn::simsens::LivoxSpec spec;
        std::mt19937_64 rng;
        std::unique_ptr<gdn::simsens::TimingModel> timing;
        rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
        struct Render {
            double t;
            std::vector<float> r;  // range per ray, inf where it hit nothing
        };
        std::deque<Render> renders;
        bool started = false;
        double t0 = 0.0, frame_start = 0.0;
        int64_t next_point = 0;  // index of the next point to fire, from the first ever
        long n_out = 0, n_lost = 0, n_points = 0, n_hits = 0;
    };

    void AddLivox(const std::string& n) {
        auto s = std::make_unique<Lidar>();
        s->name = n;
        s->frame = declare_parameter<std::string>(n + ".frame_id");
        s->lidar_id = static_cast<uint8_t>(declare_parameter<int>(n + ".lidar_id"));
        s->reflectivity = static_cast<uint8_t>(declare_parameter<int>(n + ".reflectivity"));
        std::string err;
        const std::string pat = declare_parameter<std::string>(n + ".pattern");
        if (!gdn::simsens::LoadLivoxPattern(gdn::DataPath(pat), declare_parameter<bool>(n + ".pattern_mirror_az"),
                                            &s->pattern, &err))
            ConfigError({n, ": ", err});
        constexpr double kDeg = M_PI / 180.0;
        auto& g = s->grid;
        g.nh = static_cast<int>(declare_parameter<int>(n + ".grid_h_samples"));
        g.nv = static_cast<int>(declare_parameter<int>(n + ".grid_v_samples"));
        g.h_min = declare_parameter<double>(n + ".grid_h_min_deg") * kDeg;
        g.h_max = declare_parameter<double>(n + ".grid_h_max_deg") * kDeg;
        g.v_min = declare_parameter<double>(n + ".grid_v_min_deg") * kDeg;
        g.v_max = declare_parameter<double>(n + ".grid_v_max_deg") * kDeg;
        if (g.nh < 2 || g.nv < 2 || g.h_max <= g.h_min || g.v_max <= g.v_min)
            ConfigError({n, ": the grid needs at least 2 x 2 rays and max > min"});
        auto& tm = s->tim;
        tm.points_per_packet = static_cast<int>(declare_parameter<int>(n + ".points_per_packet"));
        tm.point_dt = declare_parameter<double>(n + ".point_dt_us") * 1e-6;
        tm.packets_per_frame = static_cast<double>(declare_parameter<int>(n + ".packets_per_frame"));
        tm.frame_period = 1.0 / declare_parameter<double>(n + ".frame_hz");
        if (tm.points_per_packet < 1 || tm.point_dt * tm.points_per_packet > tm.PacketPeriod())
            ConfigError({n, ": a packet's points must fit in its period"});
        s->spec.min_range = declare_parameter<double>(n + ".min_range");
        s->spec.max_range = declare_parameter<double>(n + ".max_range");
        s->spec.range_sigma = declare_parameter<double>(n + ".range_sigma_m");
        s->spec.angle_sigma = declare_parameter<double>(n + ".angle_sigma_deg") * kDeg;
        const auto seed = static_cast<uint64_t>(declare_parameter<int>(n + ".seed"));
        s->rng.seed(seed);
        s->timing = std::make_unique<gdn::simsens::TimingModel>(ReadTiming(n), seed + 2000);
        const std::string topic = declare_parameter<std::string>(n + ".topic");
        s->pub = create_publisher<livox_ros_driver2::msg::CustomMsg>(topic, rclcpp::QoS(10));
        Lidar* p = s.get();
        s->sub = create_subscription<sensor_msgs::msg::PointCloud2>(
            declare_parameter<std::string>(n + ".raw_topic"), rclcpp::QoS(5),
            [this, p](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) { OnCloud(p, *msg); });
        RCLCPP_INFO(get_logger(), "%s: %s (%s), pattern %s (%zu directions), %d x %d rays, %.0f Hz frames", n.c_str(),
                    topic.c_str(), s->frame.c_str(), pat.c_str(), s->pattern.az.size(), g.nh, g.nv,
                    1.0 / tm.frame_period);
        lidars_.push_back(std::move(s));
    }

    // One Gazebo render: the range of every ray. A frame goes out once Gazebo's time passes
    // its end (TickLidar, from here and the flush timer); its last points, after the
    // latest render, are extrapolated from the last two renders.
    void OnCloud(Lidar* s, const sensor_msgs::msg::PointCloud2& raw) {
        const double t = Sec(raw.header.stamp);
        ObserveClock(t);
        const auto& g = s->grid;
        const bool xyz = raw.fields.size() >= 3 && raw.fields[0].name == "x" && raw.fields[0].offset == 0 &&
                         raw.fields[1].offset == 4 && raw.fields[2].offset == 8 &&
                         raw.fields[0].datatype == sensor_msgs::msg::PointField::FLOAT32;
        if (!xyz || static_cast<int>(raw.width) != g.nh || static_cast<int>(raw.height) != g.nv || raw.is_bigendian ||
            raw.data.size() < static_cast<size_t>(raw.point_step) * raw.width * raw.height) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                  "%s: raw cloud is %u x %u, want %d x %d with x, y, z float32 first", s->name.c_str(),
                                  raw.width, raw.height, g.nh, g.nv);
            return;
        }
        Lidar::Render rd{t, std::vector<float>(static_cast<size_t>(g.nh) * g.nv)};
        for (size_t k = 0; k < rd.r.size(); ++k) {
            float v[3];
            std::memcpy(v, raw.data.data() + k * raw.point_step, sizeof v);
            const float r = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            rd.r[k] = std::isfinite(r) ? r : std::numeric_limits<float>::infinity();
        }
        s->renders.push_back(std::move(rd));
        while (s->renders.size() > 6) s->renders.pop_front();
        if (!s->started) {
            s->started = true;
            s->t0 = s->frame_start = t;
        }
        TickLidar(s, t);
    }

    void TickLidar(Lidar* s, double now_sim) {
        if (!s->started || s->renders.size() < 2) return;
        while (s->frame_start + s->tim.frame_period <= now_sim + 1e-9) {
            EmitLidarFrame(s, now_sim);
            s->frame_start += s->tim.frame_period;
        }
    }

    void EmitLidarFrame(Lidar* s, double t_now) {
        const double end = s->frame_start + s->tim.frame_period;
        const int64_t k0 = s->next_point, k1 = s->tim.FirstPointAtOrAfter(s->t0, end);
        s->next_point = k1;
        if (k1 <= k0) return;
        const double t_first = s->tim.PointTime(s->t0, k0);
        const gdn::simsens::Message msg = s->timing->Apply(end, t_now);
        if (msg.lost) {
            ++s->n_lost;
            return;
        }
        const double stamp = clock_.ToStamp(t_first + (msg.stamp - end));  // the first point's time (+ jitter)
        auto out = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
        out->header.stamp = ToMsg(stamp);
        out->header.frame_id = s->frame;
        out->timebase = static_cast<uint64_t>(out->header.stamp.sec) * 1000000000ULL + out->header.stamp.nanosec;
        out->lidar_id = s->lidar_id;
        out->rsvd = {0, 0, 0};
        out->points.resize(static_cast<size_t>(k1 - k0));
        const auto& rs = s->renders;
        const auto npat = static_cast<int64_t>(s->pattern.az.size());
        size_t b = 0;  // the last render at or before the point's time
        for (int64_t k = k0; k < k1; ++k) {
            const double tp = s->tim.PointTime(s->t0, k);
            while (b + 1 < rs.size() && rs[b + 1].t <= tp) ++b;
            // The renders either side, or past the latest one, the last two.
            const size_t a = b + 1 < rs.size() ? b + 1 : b;
            const size_t b2 = a == b && b > 0 ? b - 1 : b;
            const int64_t pk = k % npat;
            const double az = s->pattern.az[pk], el = s->pattern.el[pk];
            auto& pt = out->points[static_cast<size_t>(k - k0)];
            pt.offset_time = static_cast<uint32_t>(std::lround((tp - t_first) * 1e9));
            pt.line = static_cast<uint8_t>(k % 4);
            pt.tag = 0;
            int idx[4];
            double w[4], rr[4];
            if (!s->grid.Around(az, el, idx, w)) continue;
            for (int m = 0; m < 4; ++m) {
                const auto c = static_cast<size_t>(idx[m]);
                rr[m] = gdn::simsens::InterpRange(tp, rs[b2].t, rs[b2].r[c], rs[a].t, rs[a].r[c]);
            }
            const double r = gdn::simsens::GridRange(rr, w);
            const gdn::simsens::LivoxPointOut q = gdn::simsens::LivoxReturn(s->spec, az, el, r, s->rng);
            if (!q.hit) continue;
            pt.x = q.x;
            pt.y = q.y;
            pt.z = q.z;
            pt.reflectivity = s->reflectivity;
            ++s->n_hits;
        }
        out->point_num = static_cast<uint32_t>(out->points.size());
        s->n_points += static_cast<long>(out->points.size());
        ++s->n_out;
        auto pub = s->pub;
        Queue(msg.arrival, [pub, out] { pub->publish(*out); });
    }
#endif

    void ObserveClock(double sim_t) {
        const bool first = !clock_.Ready();
        clock_.Observe(sim_t, now().seconds());
        if (first) RCLCPP_INFO(get_logger(), "clock: stamps are Gazebo time + %.6f s", clock_.Offset());
    }

    // arrival: on Gazebo's clock.
    void Queue(double arrival, std::function<void()> f) { q_.push(Pending{arrival, order_++, std::move(f)}); }

    void Flush() {
        if (!clock_.Ready()) return;
        const double now_s = clock_.Now(now().seconds());
        for (auto& r : ranges_) TickRange(r.get(), now_s);
#ifdef GDN_HAVE_LIVOX
        for (auto& l : lidars_) TickLidar(l.get(), now_s);
#endif
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
        for (auto& s : stereos_) {
            char gain[32];
            std::snprintf(gain, sizeof gain, "%.2f", s->ae->Gain());
            msg += " " + s->name + " " + std::to_string(s->n_out / 10) + " Hz pairs (" + std::to_string(s->n_lost) +
                   " lost, " + std::to_string(s->n_unpaired) + " unpaired, gain " + gain + ")";
            s->n_out = s->n_lost = s->n_unpaired = 0;
        }
#ifdef GDN_HAVE_LIVOX
        for (auto& s : lidars_) {
            const long pts = s->n_out > 0 ? s->n_points / s->n_out : 0;
            const long hits = s->n_points > 0 ? 100 * s->n_hits / s->n_points : 0;
            msg += " " + s->name + " " + std::to_string(s->n_out / 10) + " Hz (" + std::to_string(s->n_lost) +
                   " lost, " + std::to_string(pts) + " points, " + std::to_string(hits) + "% returns)";
            s->n_out = s->n_lost = s->n_points = s->n_hits = 0;
        }
#endif
        if (!clock_.Ready()) msg += "; no Gazebo sensor data yet (is the bridge running?)";
        RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    }

    gdn::simsens::SimClock clock_;
    std::vector<std::unique_ptr<Imu>> imus_;
    std::vector<std::unique_ptr<Range>> ranges_;
    std::vector<std::unique_ptr<Stereo>> stereos_;
#ifdef GDN_HAVE_LIVOX
    std::vector<std::unique_ptr<Lidar>> lidars_;
#else
    void AddLivox(const std::string& n) {
        ConfigError({n,
                     ": built without livox_ros_driver2's messages (source /opt/livox_msgs/setup.bash or a "
                     "Livox driver workspace, then rebuild gdn_fusion)"});
    }
#endif
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
