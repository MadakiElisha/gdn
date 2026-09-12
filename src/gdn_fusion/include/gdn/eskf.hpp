#pragma once
// 15-state error-state Kalman filter core for terrain-referenced navigation.
// ROS-free (pure Eigen). Formulation per Sola arXiv:1711.02508; gating and
// Joseph-form update per Bar-Shalom et al. (2001). Validated by selftest.cpp
// and tools/bench.sh (TEST-009/010). Derivations: docs/design/eskf_design.md.
#include <Eigen/Dense>
#include <cmath>
#include <utility>
#include "gdn/map_db.hpp"

namespace gdn {

struct EskfConfig {
    double r_trn = 400.0;          // (20 m)^2: sensor noise + DB mismatch (robustness)
    double slope_min = 0.02;       // observability gate [m/m]
    double gate_sigma = 3.0;       // innovation gate [sigma]
    double gate_abs_m = 40.0;      // innovation gate [m]
    double clamp_dtheta_deg = 0.2; // per-update attitude injection limit
    double clamp_dp = 5.0;         // per-update position injection limit (m)
    double clamp_dv = 0.5;         // per-update velocity injection limit (OI-006)
    double clamp_dva = 0.005;      // per-update accel-bias injection limit
    double clamp_dvg_deg = 0.05;   // per-update gyro-bias injection limit
    double still_rate = 0.05;      // static detector: per-sample motion gate [rad/s]
    int    static_n = 500;         // static detector: samples required
    double max_gyro_mean = 0.02;   // static detector: |mean rate| limit [rad/s]
    double max_grav_dev = 0.5;     // static detector: | |a| - g | limit [m/s^2]
    double max_accel_var = 1.0;    // static detector: per-axis variance limit
};

class Eskf {
public:
    using V3  = Eigen::Vector3d;
    using V15 = Eigen::Matrix<double, 15, 1>;
    using M15 = Eigen::Matrix<double, 15, 15>;
    static constexpr double kG = 9.80665;
    static constexpr double kDeg = M_PI / 180.0;
    enum class UpdResult { kApplied, kSlopeGate, kInnovGate };

    explicit Eskf(EskfConfig cfg = EskfConfig{}) : cfg_(std::move(cfg)) {
        g_ = V3(0, 0, kG);
        q_ = Eigen::Quaterniond::Identity();
        pos_ = vel_ = ba_ = bg_ = V3::Zero();
        x_.setZero(); P_.setZero(); Qc_.setZero();
        sa_ = sw_ = sa2_ = V3::Zero();
        Qc_.block<3,3>(3,3).diagonal().setConstant(0.02 * 0.02);
        Qc_.block<3,3>(6,6).diagonal().setConstant((0.02*kDeg) * (0.02*kDeg));
        Qc_.block<3,3>(9,9).diagonal().setConstant(5e-4 * 5e-4);
        Qc_.block<3,3>(12,12).diagonal().setConstant((0.01*kDeg) * (0.01*kDeg));
    }

    // Static collection: motion-excluding, vibration-averaging. True once aligned.
    bool FeedStatic(const V3& a_m, const V3& w_m) {
        if (aligned_) return true;
        if (w_m.norm() < cfg_.still_rate) { n_++; sa_ += a_m; sw_ += w_m; sa2_ += a_m.cwiseAbs2(); }
        else { n_ = 0; sa_ = sw_ = sa2_ = V3::Zero(); }
        if (n_ < cfg_.static_n) return false;
        const V3 ma = sa_ / n_, mw = sw_ / n_;
        const V3 var = sa2_ / n_ - ma.cwiseAbs2();
        if (mw.norm() > cfg_.max_gyro_mean || std::abs(ma.norm() - kG) > cfg_.max_grav_dev ||
            var.maxCoeff() > cfg_.max_accel_var) {
            n_ = 0; sa_ = sw_ = sa2_ = V3::Zero(); return false;
        }
        bg_ = mw;
        const double roll  = std::atan2(-ma.y(), -ma.z());
        const double pitch = std::atan2( ma.x(), std::hypot(ma.y(), ma.z()));
        q_ = Eigen::AngleAxisd(pitch, V3::UnitY()) * Eigen::AngleAxisd(roll, V3::UnitX());
        ba_ = ma - (q_.conjugate() * (-g_));
        ResetCovariance(0.0);
        aligned_ = true;
        return true;
    }

    // Strapdown propagation + covariance growth; sub-stepped, stall-tolerant.
    void Propagate(const V3& a_m, const V3& w_m, double dt) {
        if (!aligned_ || dt <= 0) return;
        if (dt > 1.0) dt = 1.0;
        const V3 a_b = a_m - ba_, w_b = w_m - bg_;
        const int n = static_cast<int>(std::ceil(dt / 0.01));
        const double h = dt / n;
        for (int k = 0; k < n; ++k) {
            const Eigen::Matrix3d Rm = q_.toRotationMatrix();
            M15 F = M15::Zero();
            F.block<3,3>(0,3)  = Eigen::Matrix3d::Identity();
            F.block<3,3>(3,6)  = -Rm * Skew(a_b);
            F.block<3,3>(3,9)  = -Rm;
            F.block<3,3>(6,6)  = -Skew(w_b);
            F.block<3,3>(6,12) = -Eigen::Matrix3d::Identity();
            const M15 Phi = M15::Identity() + F * h;
            P_ = Repair(Phi * P_ * Phi.transpose() + Qc_ * h);
            const double ang = w_b.norm() * h;
            if (ang > 1e-9)
                q_ = (q_ * Eigen::Quaterniond(Eigen::AngleAxisd(ang, w_b.normalized()))).normalized();
            vel_ += (q_ * a_b + g_) * h;
            pos_ += vel_ * h;
        }
    }

    // Heading transfer = re-initialization: rotate nominal, reset error state.
    void TransferAlignYaw(double yaw_rad) {
        const Eigen::Quaterniond rz(Eigen::AngleAxisd(yaw_rad, V3::UnitZ()));
        const Eigen::Matrix3d Rz = rz.toRotationMatrix();
        q_ = rz * q_; pos_ = Rz * pos_; vel_ = Rz * vel_;
        x_.setZero();
        ResetCovariance(1.0 * kDeg);
        yaw_aligned_ = true;
    }

    // Scalar terrain-elevation update with observability + innovation gates.
    UpdResult ApplyTrn(double meas, const MapQuery& mq,
                       double* innov_out = nullptr, double* s_out = nullptr) {
        if (!aligned_ || !yaw_aligned_ || !mq.ok) return UpdResult::kSlopeGate;
        if (std::hypot(mq.dn, mq.de) < cfg_.slope_min) return UpdResult::kSlopeGate;
        Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
        H(0, 0) = mq.dn; H(0, 1) = mq.de;
        const double innov = meas - mq.h - H * x_;
        const double S = (H * P_ * H.transpose())(0, 0) + cfg_.r_trn;
        if (innov_out) *innov_out = innov;
        if (s_out) *s_out = S;
        if (S <= 0 || std::abs(innov) / std::sqrt(S) > cfg_.gate_sigma ||
            std::abs(innov) > cfg_.gate_abs_m) { n_rej_++; return UpdResult::kInnovGate; }
        const Eigen::Matrix<double, 15, 1> K = (P_ * H.transpose()) / S;
        V15 dx = K * innov;
        dx.segment<3>(3)  = Clamp(dx.segment<3>(3),  cfg_.clamp_dv);
        dx.segment<3>(6)  = Clamp(dx.segment<3>(6),  cfg_.clamp_dtheta_deg * kDeg);
        dx.segment<3>(9)  = Clamp(dx.segment<3>(9),  cfg_.clamp_dva);
        dx.segment<3>(12) = Clamp(dx.segment<3>(12), cfg_.clamp_dvg_deg * kDeg);
        x_ += dx;
        const M15 IKH = M15::Identity() - K * H;
        P_ = Repair(IKH * P_ * IKH.transpose() + cfg_.r_trn * K * K.transpose());
        pos_ += x_.segment<3>(0); vel_ += x_.segment<3>(3);
        const double da = x_.segment<3>(6).norm();
        if (da > 1e-9)
            q_ = (q_ * Eigen::Quaterniond(Eigen::AngleAxisd(da, x_.segment<3>(6).normalized()))).normalized();
        ba_ += x_.segment<3>(9); bg_ += x_.segment<3>(12);
        M15 J = M15::Identity();
        J.block<3,3>(6,6) = Eigen::Matrix3d::Identity() - Skew(x_.segment<3>(6));
        P_ = Repair(J * P_ * J.transpose());
        x_.setZero();
        n_upd_++;
        return UpdResult::kApplied;
    }

    const V3& pos() const { return pos_; }
    const V3& vel() const { return vel_; }
    const Eigen::Quaterniond& q() const { return q_; }
    const V3& accel_bias() const { return ba_; }
    const V3& gyro_bias() const { return bg_; }
    double sigma_pos() const { return std::sqrt(P_(0, 0) + P_(1, 1)); }
    double sigma_att_deg() const {
        return std::sqrt(std::max({P_(6,6), P_(7,7), P_(8,8)})) / kDeg; }
    bool aligned() const { return aligned_; }
    bool yaw_aligned() const { return yaw_aligned_; }
    int updates() const { return n_upd_; }
    int rejects() const { return n_rej_; }

private:
    void ResetCovariance(double yaw_bump_rad) {
        P_.setZero();
        P_.block<3,3>(0,0).diagonal().setConstant(100.0);
        P_.block<3,3>(3,3).diagonal().setConstant(1.0);
        P_.block<3,3>(6,6).diagonal().setConstant((2.0*kDeg) * (2.0*kDeg));
        if (yaw_bump_rad > 0) P_(8, 8) += yaw_bump_rad * yaw_bump_rad;
        P_.block<3,3>(9,9).diagonal().setConstant(0.01);
        P_.block<3,3>(12,12).diagonal().setConstant((0.1*kDeg) * (0.1*kDeg));
    }
    static Eigen::Matrix3d Skew(const V3& v) {
        Eigen::Matrix3d S; S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0; return S; }
    static V3 Clamp(const V3& v, double lim) {
        const double n = v.norm(); return (n > lim) ? v * (lim / n) : v; }
    static M15 Repair(const M15& P) {
        Eigen::LLT<M15> llt(P);
        if (llt.info() == Eigen::Success) return P;
        Eigen::SelfAdjointEigenSolver<M15> es(P);
        return es.eigenvectors() * es.eigenvalues().cwiseMax(1e-9).asDiagonal() *
               es.eigenvectors().transpose(); }

    EskfConfig cfg_;
    V3 g_, pos_, vel_, ba_, bg_, sa_, sw_, sa2_;
    Eigen::Quaterniond q_;
    V15 x_; M15 P_, Qc_;
    int n_ = 0, n_upd_ = 0, n_rej_ = 0;
    bool aligned_ = false, yaw_aligned_ = false;
};

}  // namespace gdn
