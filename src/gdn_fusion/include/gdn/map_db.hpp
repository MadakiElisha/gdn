#pragma once
// Binary terrain database (REQ-DB-001): 40-byte header + row-major float32 grid.
// Frame convention: local NED metres around an origin, by default the grid centre
// (from the header). SetOrigin() puts two maps in one frame (R1.3): a truth world map
// and an onboard map cut, sampled or sourced differently have different centres, and
// without a shared origin the same local coordinate names different places.
#include <cstdint>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

namespace gdn {

struct MapQuery { double h = 0.0, dn = 0.0, de = 0.0; bool ok = false; };

class MapDb {
public:
    static constexpr double kMPerDegLat = 111320.0;

    bool Load(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) return false;
        int32_t nx = 0, ny = 0;
        f.read(reinterpret_cast<char*>(&nx), 4);
        f.read(reinterpret_cast<char*>(&ny), 4);
        f.read(reinterpret_cast<char*>(&lat0_), 8); f.read(reinterpret_cast<char*>(&lon0_), 8);
        f.read(reinterpret_cast<char*>(&dlat_), 8); f.read(reinterpret_cast<char*>(&dlon_), 8);
        if (!f || nx <= 2 || ny <= 2 || dlat_ == 0.0 || dlon_ == 0.0) return false;
        // Reject truncated / padded files: size must match the header exactly.
        f.seekg(0, std::ios::end);
        if (static_cast<long long>(f.tellg()) != 40LL + 4LL * nx * ny) return false;
        f.seekg(40, std::ios::beg);
        nx_ = nx; ny_ = ny;
        alt_.resize(static_cast<size_t>(nx_) * ny_);
        f.read(reinterpret_cast<char*>(alt_.data()), 4LL * nx_ * ny_);
        SetOrigin(lat0_ + 0.5 * ny_ * dlat_, lon0_ + 0.5 * nx_ * dlon_);
        return f.good();
    }

    // Local-frame origin (degrees). The flat-earth scale comes from the origin's
    // latitude, so two maps with the same origin share the exact same projection.
    void SetOrigin(double lat_deg, double lon_deg) {
        lat_c_ = lat_deg; lon_c_ = lon_deg;
        m_per_deg_lon_ = kMPerDegLat * std::cos(lat_c_ * 3.141592653589793 / 180.0);
    }
    double grid_centre_lat() const { return lat0_ + 0.5 * ny_ * dlat_; }
    double grid_centre_lon() const { return lon0_ + 0.5 * nx_ * dlon_; }

    // Bilinear altitude [m] + central-difference gradient [m/m] at NED meters.
    MapQuery Query(double north_m, double east_m) const {
        MapQuery q;
        const double fi = (lat_c_ + north_m / kMPerDegLat - lat0_) / dlat_;
        const double fj = (lon_c_ + east_m / m_per_deg_lon_ - lon0_) / dlon_;
        if (fi < 1.0 || fi > ny_ - 2.001 || fj < 1.0 || fj > nx_ - 2.001) return q;
        const int i = static_cast<int>(fi), j = static_cast<int>(fj);
        const double ti = fi - i, tj = fj - j;
        q.h  = (1-ti)*((1-tj)*A(i,j)   + tj*A(i,j+1)) + ti*((1-tj)*A(i+1,j)   + tj*A(i+1,j+1));
        // Gradient = slope of this same bilinear surface at (ti, tj) (R1.4). Central
        // differences at the cell corner disagreed with h by up to 2 degrees of slope
        // on Zermatt-like terrain, so the filter's H didn't match its own prediction.
        // This slope is piecewise linear and jumps at cell edges, as the surface does.
        q.dn = ((1-tj)*(A(i+1,j) - A(i,j)) + tj*(A(i+1,j+1) - A(i,j+1))) / (dlat_ * kMPerDegLat);
        q.de = ((1-ti)*(A(i,j+1) - A(i,j)) + ti*(A(i+1,j+1) - A(i+1,j))) / (dlon_ * m_per_deg_lon_);
        q.ok = true;
        return q;
    }

    int rows() const { return ny_; }
    int cols() const { return nx_; }
    float At(int i, int j) const { return A(i, j); }
    double lat0() const { return lat0_; } double lon0() const { return lon0_; }
    double lat_c() const { return lat_c_; } double lon_c() const { return lon_c_; }
    double m_per_deg_lon() const { return m_per_deg_lon_; }
    // Local north/east (m) of grid sample (i, j), the same geometry Query() uses.
    double NorthOf(int i) const { return (lat0_ + i * dlat_ - lat_c_) * kMPerDegLat; }
    double EastOf(int j) const { return (lon0_ + j * dlon_ - lon_c_) * m_per_deg_lon_; }
    double dlat() const { return dlat_; } double dlon() const { return dlon_; }

private:
    double A(int i, int j) const { return alt_[static_cast<size_t>(i) * nx_ + j]; }
    std::vector<float> alt_;
    int nx_ = 0, ny_ = 0;
    double lat_c_ = 0, lon_c_ = 0, m_per_deg_lon_ = kMPerDegLat;
    double lat0_ = 0, lon0_ = 0, dlat_ = 1, dlon_ = 1;
};

}  // namespace gdn
