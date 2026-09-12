// Visualization node: terrain as PointCloud2 (height-colored) + estimate path.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "gdn/map_db.hpp"

class MapVizNode : public rclcpp::Node {
public:
    MapVizNode() : Node("gdn_map_viz") {
        const std::string map_path = declare_parameter("map_path",
            std::string("/home/madakie/gdn_workspace/data/maps/terrain_db_sigma4.bin"));
        if (!map_.Load(map_path)) {
            RCLCPP_ERROR(get_logger(), "Failed to load map: %s", map_path.c_str());
            throw std::runtime_error("map load failed");
        }
        base_alt_ = map_.At(map_.rows() / 2, map_.cols() / 2);
        RCLCPP_INFO(get_logger(), "Loaded map %dx%d (base_alt=%.1f m)",
                    map_.rows(), map_.cols(), base_alt_);

        rclcpp::QoS latch(1);
        latch.reliability(rclcpp::ReliabilityPolicy::Reliable);
        latch.durability(rclcpp::DurabilityPolicy::TransientLocal);
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("terrain_cloud", latch);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("terrain_mesh", latch);
        path_pub_  = create_publisher<nav_msgs::msg::Path>("gdn_path", 10);

        // Republish terrain periodically so late joiners (RViz) always get it.
        cloud_timer_ = create_wall_timer(std::chrono::seconds(2),
                                         [this]() { publishTerrainCloud(); publishTerrainMesh(); });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/gdn/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = msg->header;
                pose.pose = msg->pose.pose;
                // Vertical channel is unobservable (OI-002): project path at
                // nominal AGL above the terrain so the horizontal track is legible.
                const double relh = map_.Query(pose.pose.position.x,
                                               pose.pose.position.y).h - base_alt_;
                pose.pose.position.z = relh + 80.0;
                path_.poses.push_back(pose);
                path_.header = msg->header;
                path_pub_->publish(path_);
            });
    }

private:
    void publishTerrainCloud() {
        const int step = 4;                       // decimated grid
        const int rows = map_.rows(), cols = map_.cols();
        std::vector<float> pts;
        pts.reserve(static_cast<size_t>(rows / step + 1) * (cols / step + 1) * 3);
        for (int i = 0; i < rows; i += step)
            for (int j = 0; j < cols; j += step) {
                pts.push_back(static_cast<float>((i - rows / 2.0) * map_.dlat() * 111320.0));
                pts.push_back(static_cast<float>((j - cols / 2.0) * map_.dlon() * 111320.0 * 0.676876));
                pts.push_back(static_cast<float>(map_.At(i, j) - base_alt_));
            }
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = "map";
        cloud.header.stamp = now();
        cloud.is_dense = true;
        cloud.is_bigendian = false;
        sensor_msgs::PointCloud2Modifier mod(cloud);
        mod.setPointCloud2FieldsByString(1, "xyz");
        mod.resize(pts.size() / 3);
        sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
        for (size_t k = 0; k < pts.size() / 3; ++k, ++ix, ++iy, ++iz) {
            *ix = pts[3 * k]; *iy = pts[3 * k + 1]; *iz = pts[3 * k + 2];
        }
        cloud_pub_->publish(cloud);
    }

    void publishTerrainMesh() {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map"; m.header.stamp = now();
        m.ns = "terrain"; m.id = 0; m.action = m.ADD;
        m.type = m.TRIANGLE_LIST;
        m.scale.x = m.scale.y = m.scale.z = 1.0;
        m.color.r = 0.7; m.color.g = 0.5; m.color.b = 0.3; m.color.a = 1.0;
        const int step = 8; const int rows = map_.rows(), cols = map_.cols();
        for (int i = 0; i + step < rows; i += step)
            for (int j = 0; j + step < cols; j += step) {
                auto P = [&](int ii, int jj) {
                    geometry_msgs::msg::Point p;
                    p.x = (ii - rows / 2.0) * map_.dlat() * 111320.0;
                    p.y = (jj - cols / 2.0) * map_.dlon() * 111320.0 * 0.676876;
                    p.z = map_.At(ii, jj) - base_alt_;
                    return p; };
                m.points.push_back(P(i, j)); m.points.push_back(P(i + step, j)); m.points.push_back(P(i, j + step));
                m.points.push_back(P(i + step, j)); m.points.push_back(P(i + step, j + step)); m.points.push_back(P(i, j + step));
            }
        marker_pub_->publish(m);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    gdn::MapDb map_;
    double base_alt_ = 0.0;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr cloud_timer_;
    nav_msgs::msg::Path path_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapVizNode>());
    rclcpp::shutdown();
    return 0;
}
