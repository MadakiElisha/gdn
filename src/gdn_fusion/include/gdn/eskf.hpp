#pragma once
// 19-state error-state Kalman filter core for terrain-referenced navigation.
// Adds barometer bias (16) and magnetometer bias (17-19) states.
// ROS-free (pure Eigen). Formulation per Sola arXiv:1711.02508.
#include <Eigen/Dense>
#include <cmath>
#include <utility>
#include <algorithm>
#include "gdn/map_db.hpp"

namespace gdn {

struct EskfConfig {
    double r_trn = 400.0;
    double slope_min = 0.02;
    double gate_sigma = 3.0;
    double gate_abs_m = 40.0;
    double clamp_dtheta_deg = 180.0;  // Relaxed to allow coarse mag yaw alignment without covariance collapse  // relaxed: 0.2 deg broke mag convergence
    double clamp_dp = 5.0;
    double clamp_dv = 0.5;
    double clamp_dva = 0.005;
    double clamp_dvg_deg = 0.05;
    double clamp_dvm = 5.0;      // uT per update (mag bias clamp)
    double still_rate = 0.05;
    int    static_n = 500;
    double max_gyro_mean = 0.02;
    double max_grav_dev = 0.5;
    double max_accel_var = 1.0;
    // Barometer (OI-002)
    double q_baro_bias = 0.001;
    double p0_baro_bias = 25.0;
    double r_baro = 4.0;
    double gate_baro_sigma = 5.0;
    // Magnetometer (OI-003)
    double q_mag_bias = 1e-5;     // (uT/s)^2/s : mag bias random walk PSD
    double p0_mag_bias = 100.0;   // uT^2 : initial mag bias variance (sigma=10uT)
    double r_mag = 4.0;           // uT^2 : mag measurement noise (sigma=2uT)
    double gate_mag_sigma = 5.0;  // 5-sigma innovation gate for mag (Mahalanobis)
    double mag_ref_north = 21.5;  // WMM Zurich  // uT : reference field N
    double mag_ref_east = 1.3;   // WMM Zurich    // uT : reference field E
    double mag_ref_down = 42.3;  // WMM Zurich   // uT : reference field D
};

class Eskf {
public:
    using V3  = Eigen::Vector3d;
    using V19 = Eigen::Matrix<double, 19, 1>;
    using M19 = Eigen::Matrix<double, 19, 19>;
    using H19 = Eigen::Matrix<double, 1, 19>;
    static constexpr double kG = 9.80665;
    static constexpr double kDeg = M_PI / 180.0;
    enum class UpdResult { kApplied, kSlopeGate, kInnovGate };

    explicit Eskf(EskfConfig cfg = EskfConfig{}) : cfg_(std::move(cfg)) {
        g_ = V3(0, 0, kG);
        q_ = Eigen::Quaterniond::Identity();
        pos_ = vel_ = ba_ = bg_ = V3::Zero();
        baro_bias_ = 0.0;
        mag_bias_ = V3::Zero();
        x_.setZero(); P_.setZero(); Qc_.setZero();
        sa_ = sw_ = sa2_ = V3::Zero();
        Qc_.block<3,3>(3,3).diagonal().setConstant(0.02 * 0.02);
        Qc_.block<3,3>(6,6).diagonal().setConstant((0.02*kDeg) * (0.02*kDeg));
        Qc_.block<3,3>(9,9).diagonal().setConstant(5e-4 * 5e-4);
        Qc_.block<3,3>(12,12).diagonal().setConstant((0.01*kDeg) * (0.01*kDeg));
        Qc_(15,15) = cfg_.q_baro_bias;
        Qc_.block<3,3>(16,16).diagonal().setConstant(cfg_.q_mag_bias);
    }

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

    void Propagate(const V3& a_m, const V3& w_m, double dt) {
        if (!aligned_ || dt <= 0) return;
        if (dt > 1.0) dt = 1.0;
        const V3 a_b = a_m - ba_, w_b = w_m - bg_;
        const int n = static_cast<int>(std::ceil(dt / 0.01));
        const double h = dt / n;
        for (int k = 0; k < n; ++k) {
            const Eigen::Matrix3d Rm = q_.toRotationMatrix();
            M19 F = M19::Zero();
            F.block<3,3>(0,3)  = Eigen::Matrix3d::Identity();
            F.block<3,3>(3,6)  = -Rm * Skew(a_b);
            F.block<3,3>(3,9)  = -Rm;
            F.block<3,3>(6,6)  = -Skew(w_b);
            F.block<3,3>(6,12) = -Eigen::Matrix3d::Identity();
            const M19 Phi = M19::Identity() + F * h;
            P_ = Repair(Phi * P_ * Phi.transpose() + Qc_ * h);
            const double ang = w_b.norm() * h;
            if (ang > 1e-9)
                q_ = (q_ * Eigen::Quaterniond(Eigen::AngleAxisd(ang, w_b.normalized()))).normalized();
            vel_ += (q_ * a_b + g_) * h;
            pos_ += vel_ * h;
        }
    }

    void TransferAlignYaw(double yaw_rad) {
        const Eigen::Quaterniond rz(Eigen::AngleAxisd(yaw_rad, V3::UnitZ()));
        const Eigen::Matrix3d Rz = rz.toRotationMatrix();
        q_ = rz * q_; pos_ = Rz * pos_; vel_ = Rz * vel_;
        x_.setZero();
        ResetCovariance(1.0 * kDeg);
        yaw_aligned_ = true;
    }

    UpdResult ApplyTrn(double meas, const MapQuery& mq,
                       double* innov_out = nullptr, double* s_out = nullptr) {
        if (!aligned_ || !yaw_aligned_ || !mq.ok) return UpdResult::kSlopeGate;
        if (std::hypot(mq.dn, mq.de) < cfg_.slope_min) return UpdResult::kSlopeGate;
        H19 H = H19::Zero();
        H(0, 0) = mq.dn; H(0, 1) = mq.de;
        const double innov = meas - mq.h - H * x_;
        const double S = (H * P_ * H.transpose())(0, 0) + cfg_.r_trn;
        if (innov_out) *innov_out = innov;
        if (s_out) *s_out = S;
        if (S <= 0 || std::abs(innov) / std::sqrt(S) > cfg_.gate_sigma ||
            std::abs(innov) > cfg_.gate_abs_m) { n_rej_++; return UpdResult::kInnovGate; }
        const V19 K = (P_ * H.transpose()) / S;
        V19 dx = K * innov;
        dx.segment<3>(3)  = Clamp(dx.segment<3>(3),  cfg_.clamp_dv);
        dx.segment<3>(6)  = Clamp(dx.segment<3>(6),  cfg_.clamp_dtheta_deg * kDeg);
        dx.segment<3>(9)  = Clamp(dx.segment<3>(9),  cfg_.clamp_dva);
        dx.segment<3>(12) = Clamp(dx.segment<3>(12), cfg_.clamp_dvg_deg * kDeg);
        dx(15) = 0.0; dx.segment<3>(16).setZero();
        x_ += dx;
        const M19 IKH = M19::Identity() - K * H;
        P_ = Repair(IKH * P_ * IKH.transpose() + cfg_.r_trn * K * K.transpose());
        pos_ += x_.segment<3>(0); vel_ += x_.segment<3>(3);
        const double da = x_.segment<3>(6).norm();
        if (da > 1e-9)
            q_ = (q_ * Eigen::Quaterniond(Eigen::AngleAxisd(da, x_.segment<3>(6).normalized()))).normalized();
        ba_ += x_.segment<3>(9); bg_ += x_.segment<3>(12);
        M19 J = M19::Identity();
        J.block<3,3>(6,6) = Eigen::Matrix3d::Identity() - Skew(x_.segment<3>(6));
        P_ = Repair(J * P_ * J.transpose());
        x_.setZero();
        n_upd_++;
        return UpdResult::kApplied;
    }

    UpdResult ApplyBaro(double z_meas) {
        if (!aligned_) return UpdResult::kSlopeGate;
        const double z_hat = pos_.z() + baro_bias_;
        const double innov = z_meas - z_hat;
        H19 H = H19::Zero();
        H(0, 2) = 1.0;
        H(0, 15) = 1.0;
        const double S = (H * P_ * H.transpose())(0, 0) + cfg_.r_baro;
        if (S <= 1e-12) return UpdResult::kInnovGate;
        if (std::abs(innov) / std::sqrt(S) > cfg_.gate_baro_sigma) {
            n_rej_++; return UpdResult::kInnovGate;
        }
        const V19 K = (P_ * H.transpose()) / S;
        V19 dx = K * innov;
        dx.segment<3>(3)  = Clamp(dx.segment<3>(3),  cfg_.clamp_dv);
        dx.segment<3>(6)  = Clamp(dx.segment<3>(6),  cfg_.clamp_dtheta_deg * kDeg);
        dx.segment<3>(9)  = Clamp(dx.segment<3>(9),  cfg_.clamp_dva);
        dx.segment<3>(12) = Clamp(dx.segment<3>(12), cfg_.clamp_dvg_deg * kDeg);
        dx.segment<3>(16).setZero();
        x_ += dx;
        const M19 IKH = M19::Identity() - K * H;
        P_ = Repair(IKH * P_ * IKH.transpose() + cfg_.r_baro * K * K.transpose());
        pos_ += x_.segment<3>(0); vel_ += x_.segment<3>(3);
        const double da = x_.segment<3>(6).norm();
        if (da > 1e-9)
            q_ = (q_ * Eigen::Quaterniond(Eigen::AngleAxisd(da, x_.segment<3>(6).normalized()))).normalized();
        ba_ += x_.segment<3>(9); bg_ += x_.segment<3>(12);
        baro_bias_ += x_(15);
        M19 J = M19::Identity();
        J.block<3,3>(6,6) = Eigen::Matrix3d::Identity() - Skew(x_.segment<3>(6));
        P_ = Repair(J * P_ * J.transpose());
        x_.setZero();
        n_upd_++;
        return UpdResult::kApplied;
    }

    // Magnetometer update (OI-003). Observes attitude and mag bias.
    UpdResult ApplyMag(const Eigen::Vector3d& z_meas) {
        if (!aligned_) return UpdResult::kSlopeGate;
        const Eigen::Matrix3d R = q_.toRotationMatrix();
        const Eigen::Vector3d m_ref(cfg_.mag_ref_north, cfg_.mag_ref_east, cfg_.mag_ref_down);
        const Eigen::Vector3d m_hat = R.transpose() * m_ref + mag_bias_;
        const Eigen::Vector3d y = z_meas - m_hat;
        
        Eigen::Matrix<double, 3, 19> H = Eigen::Matrix<double, 3, 19>::Zero();
        H.block<3,3>(0, 6) = Skew(m_hat);
        H.block<3,3>(0, 16) = Eigen::Matrix3d::Identity();
        
        const Eigen::Matrix3d S = (H * P_ * H.transpose()) + cfg_.r_mag * Eigen::Matrix3d::Identity();
        // Use LLT to check for positive-definiteness (numerical stability)
        Eigen::LLT<Eigen::Matrix3d> llt(S);
        if (llt.info() != Eigen::Success) return UpdResult::kInnovGate;
        
        const Eigen::Matrix3d S_inv = S.inverse();
        const double d2 = y.transpose() * S_inv * y;
        // Explicit NaN guard: NaN > gate is false in C++, which would bypass the gate!
        if (std::isnan(d2) || d2 > cfg_.gate_mag_sigma * cfg_.gate_mag_sigma) {
            n_rej_++; return UpdResult::kInnovGate;
        }
        
        const Eigen::Matrix<double, 19, 3> K = P_ * H.transpose() * S_inv;
        V19 dx = K * y;
        
        dx.segment<3>(3)  = Clamp(dx.segment<3>(3),  cfg_.clamp_dv);
        dx.segment<3>(6)  = Clamp(dx.segment<3>(6),  cfg_.clamp_dtheta_deg * kDeg);
        dx.segment<3>(9)  = Clamp(dx.segment<3>(9),  cfg_.clamp_dva);
        dx.segment<3>(12) = Clamp(dx.segment<3>(12), cfg_.clamp_dvg_deg * kDeg);
        dx.segment<3>(16) = Clamp(dx.segment<3>(16), cfg_.clamp_dvm);
        
        x_ += dx;
        
        const M19 IKH = M19::Identity() - K * H;
        P_ = Repair(IKH * P_ * IKH.transpose() + K * (cfg_.r_mag * Eigen::Matrix3d::Identity()) * K.transpose());
        
        pos_ += x_.segment<3>(0); vel_ += x_.segment<3>(3);
        const double da = x_.segment<3>(6).norm();
        if (da > 1e-9)
            q_ = (q_ * Eigen::Quaterniond(Eigen::AngleAxisd(da, x_.segment<3>(6).normalized()))).normalized();
        ba_ += x_.segment<3>(9); bg_ += x_.segment<3>(12);
        baro_bias_ += x_(15);
        mag_bias_ += x_.segment<3>(16);
        
        M19 J = M19::Identity();
        J.block<3,3>(6,6) = Eigen::Matrix3d::Identity() - Skew(x_.segment<3>(6));
        P_ = Repair(J * P_ * J.transpose());
        x_.setZero();
        
        n_upd_++;
        return UpdResult::kApplied;
    }

    void SetInitialZ(double z_ned) {
        if (aligned_) { pos_.z() = z_ned; x_.setZero(); }
    }
    double p_asym() const { return (P_ - P_.transpose()).norm(); }
    double p_mineig() const {
        Eigen::SelfAdjointEigenSolver<M19> es(P_);
        return es.eigenvalues().minCoeff(); }
    double p_bgz() const {
        return Eigen::Vector3d(P_(12,2), P_(13,2), P_(14,2)).norm(); }

    const V3& pos() const { return pos_; }
    const V3& vel() const { return vel_; }
    const Eigen::Quaterniond& q() const { return q_; }
    const V3& accel_bias() const { return ba_; }
    const V3& gyro_bias() const { return bg_; }
    double baro_bias() const { return baro_bias_; }
    const V3& mag_bias() const { return mag_bias_; }
    double sigma_pos() const { return std::sqrt(P_(0, 0) + P_(1, 1)); }
    double sigma_pos_z() const { return std::sqrt(P_(2, 2)); }
    double sigma_baro_bias() const { return std::sqrt(P_(15, 15)); }
    double MagInnovation(const V3& z_m) const {
        if (!aligned_) return 999.0;
        const Eigen::Matrix3d R = q_.toRotationMatrix();
        const V3 m_ref(cfg_.mag_ref_north, cfg_.mag_ref_east, cfg_.mag_ref_down);
        const V3 m_hat = R.transpose() * m_ref + mag_bias_;
        return (z_m - m_hat).norm();
    }

    double sigma_att_deg() const {
        return std::sqrt(std::max({P_(6,6), P_(7,7), P_(8,8)})) / kDeg; }
    bool aligned() const { return aligned_; }
    bool yaw_aligned() const { return yaw_aligned_; }
    void AlignYawFromMag(const V3& z_m) {
        if (!aligned_ || yaw_aligned_) return;
        const Eigen::Matrix3d R = q_.toRotationMatrix();
        const V3 m_ref(cfg_.mag_ref_north, cfg_.mag_ref_east, cfg_.mag_ref_down);
        const V3 m_hat = R.transpose() * m_ref;
        
        // Compute yaw error from horizontal components of measured vs predicted field
        double yaw_err = std::atan2(m_hat.y(), m_hat.x()) - std::atan2(z_m.y(), z_m.x());
        
        // Apply correction instantly
        const Eigen::Quaterniond dq(Eigen::AngleAxisd(yaw_err, V3::UnitZ()));
        q_ = dq * q_;
        const Eigen::Matrix3d dR = dq.toRotationMatrix();
        pos_ = dR * pos_;  // Rotate frame (will be ~0 on the ground)
        vel_ = dR * vel_;
        
        x_.setZero();
        // Reset covariance to flight-phase values (mirrors GNSS TransferAlignYaw)
        // This opens up pos/vel/gyro-bias covariances so TRN can correct takeoff transients.
        ResetCovariance(5.0 * kDeg);
        yaw_aligned_ = true;
    }

    int updates() const { return n_upd_; }
    int rejects() const { return n_rej_; }

private:
    void ResetCovariance(double yaw_bump_rad) {
        P_.setZero();
        P_.block<3,3>(0,0).diagonal().setConstant(100.0);
        P_.block<3,3>(3,3).diagonal().setConstant(1.0);
        // Pitch and Roll are heavily constrained by gravity during static alignment.
        P_(6,6) = (0.5*kDeg) * (0.5*kDeg);
        P_(7,7) = (0.5*kDeg) * (0.5*kDeg);
        // Yaw is completely unobservable from static gravity.
        P_(8,8) = (180.0*kDeg) * (180.0*kDeg);
        if (yaw_bump_rad > 0) P_(8, 8) = yaw_bump_rad * yaw_bump_rad;
        P_.block<3,3>(9,9).diagonal().setConstant(0.01);
        P_.block<3,3>(12,12).diagonal().setConstant((0.1*kDeg) * (0.1*kDeg));
        P_(15,15) = cfg_.p0_baro_bias;
        P_.block<3,3>(16,16).diagonal().setConstant(cfg_.p0_mag_bias);
    }
    static Eigen::Matrix3d Skew(const V3& v) {
        Eigen::Matrix3d S; S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0; return S; }
    static V3 Clamp(const V3& v, double lim) {
        const double n = v.norm(); return (n > lim) ? v * (lim / n) : v; }
    static M19 Repair(const M19& P) {
        Eigen::LLT<M19> llt(P);
        if (llt.info() == Eigen::Success) return P;
        Eigen::SelfAdjointEigenSolver<M19> es(P);
        return es.eigenvectors() * es.eigenvalues().cwiseMax(1e-9).asDiagonal() *
               es.eigenvectors().transpose(); }

    EskfConfig cfg_;
    V3 g_, pos_, vel_, ba_, bg_, sa_, sw_, sa2_, mag_bias_;
    double baro_bias_;
    Eigen::Quaterniond q_;
    V19 x_; M19 P_, Qc_;
    int n_ = 0, n_upd_ = 0, n_rej_ = 0;
    bool aligned_ = false, yaw_aligned_ = false;
};

}  // namespace gdn
