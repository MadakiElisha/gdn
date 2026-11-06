// Simulated static IMU log from a model file (R1.6), for checking tools/allan_fit.py
// against known parameters before it is used on the real IMUs:
//
//   imu_sim placeholder|config/imu/FILE.yaml HOURS RATE_HZ OUT.csv [SEED]
//
// Writes t[s],ax,ay,az,gx,gy,gz: level and still (specific force -g on z), corrupted by
// gdn::ImuErrorSim, the same code the Monte Carlo uses.
#include <cstdio>
#include <cstdlib>
#include <string>
#include "gdn/imu_model.hpp"
#include "gdn/paths.hpp"

int main(int argc, char** argv) {
    if (argc < 5) {
        std::printf("usage: imu_sim placeholder|MODEL.yaml HOURS RATE_HZ OUT.csv [SEED]\n");
        return 1;
    }
    gdn::ImuNoiseModel m;
    const std::string which = argv[1];
    if (which == "placeholder") {
        m = gdn::ImuPlaceholder();
    } else {
        std::string err;
        if (!gdn::LoadImuModel(gdn::DataPath(which), &m, &err)) { std::printf("ERROR: %s\n", err.c_str()); return 1; }
    }
    const double hours = std::atof(argv[2]), rate = std::atof(argv[3]);
    const uint64_t seed = argc > 5 ? std::strtoull(argv[5], nullptr, 10) : 1;
    if (hours <= 0 || rate <= 0) { std::printf("ERROR: HOURS and RATE_HZ must be > 0\n"); return 1; }
    std::FILE* f = std::fopen(argv[4], "w");
    if (!f) { std::printf("ERROR: cannot write %s\n", argv[4]); return 1; }
    gdn::ImuErrorSim sim(m, seed);
    const double dt = 1.0 / rate;
    const long n = static_cast<long>(hours * 3600.0 * rate);
    std::fprintf(f, "t,ax,ay,az,gx,gy,gz\n");
    for (long k = 0; k < n; ++k) {
        Eigen::Vector3d a(0, 0, -9.80665), w = Eigen::Vector3d::Zero();
        sim.Corrupt(&a, &w, dt);
        std::fprintf(f, "%.4f,%.7e,%.7e,%.7e,%.7e,%.7e,%.7e\n", k * dt, a.x(), a.y(), a.z(), w.x(), w.y(), w.z());
    }
    std::fclose(f);
    std::printf("imu_sim: %s, %.2f h at %.0f Hz, seed %llu -> %s\n", m.name.c_str(), hours, rate,
                static_cast<unsigned long long>(seed), argv[4]);
    return 0;
}
