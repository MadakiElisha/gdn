#pragma once
// Binary terrain database (REQ-DB-001): 40-byte header + row-major float32 grid.
// Frame convention: local NED meters around (kLatC, kLonC) == vehicle spawn.
#include <cstdint>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

namespace gdn {

struct MapQuery { double h = 0.0, dn = 0.0, de = 0.0; bool ok = false; };

class MapDb {
public:
    static constexpr double kLatC = 45.96319444444445, kLonC = 7.6440277777777785;
    static constexpr double kMPerDegLat = 111320.0;
    static constexpr double kMPerDegLon = 111320.0 * 0.695120;  // cos(kLatC)  // cos(kLatC)

    bool Load(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) return false;
        int32_t nx = 0, ny = 0;
        f.read(reinterpret_cast<char*>(&nx), 4);
        f.read(reinterpret_cast<char*>(&ny), 4);
        f.read(reinterpret_cast<char*>(&lat0_), 8); f.read(reinterpret_cast<char*>(&lon0_), 8);
        f.read(reinterpret_cast<char*>(&dlat_), 8); f.read(reinterpret_cast<char*>(&dlon_), 8);
        if (nx <= 2 || ny <= 2) return false;
        nx_ = nx; ny_ = ny;
        alt_.resize(static_cast<size_t>(nx_) * ny_);
        f.read(reinterpret_cast<char*>(alt_.data()), 4LL * nx_ * ny_);
        lat_c_ = lat0_ + 0.5 * ny_ * dlat_;
        lon_c_ = lon0_ + 0.5 * nx_ * dlon_;
        m_per_deg_lon_ = 111320.0 * std::cos(lat_c_ * 3.141592653589793 / 180.0);
        return f.good();
    }

    // Bilinear altitude [m] + central-difference gradient [m/m] at NED meters.
    MapQuery Query(double north_m, double east_m) const {
        MapQuery q;
        const double fi = (lat_c_ + north_m / kMPerDegLat - lat0_) / dlat_;
        const double fj = (lon_c_ + east_m / kMPerDegLon - lon0_) / dlon_;
        if (fi < 1.0 || fi > ny_ - 2.001 || fj < 1.0 || fj > nx_ - 2.001) return q;
        const int i = static_cast<int>(fi), j = static_cast<int>(fj);
        const double ti = fi - i, tj = fj - j;
        q.h  = (1-ti)*((1-tj)*A(i,j)   + tj*A(i,j+1)) + ti*((1-tj)*A(i+1,j)   + tj*A(i+1,j+1));
        q.dn = (A(i+1,j) - A(i-1,j)) / (2.0 * dlat_ * 111320.0);
        q.de = (A(i,j+1) - A(i,j-1)) / (2.0 * dlon_ * m_per_deg_lon_);
        q.ok = true;
        return q;
    }

    int rows() const { return ny_; }
    int cols() const { return nx_; }
    float At(int i, int j) const { return A(i, j); }
    double lat0() const { return lat0_; } double lon0() const { return lon0_; }
    double dlat() const { return dlat_; } double dlon() const { return dlon_; }

private:
    double A(int i, int j) const { return alt_[static_cast<size_t>(i) * nx_ + j]; }
    std::vector<float> alt_;
    int nx_ = 0, ny_ = 0;
    double lat_c_, lon_c_, m_per_deg_lon_;
    double lat0_ = 0, lon0_ = 0, dlat_ = 1, dlon_ = 1;
};

}  // namespace gdn
