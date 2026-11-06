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
    // Altimeter / height-above-ground (R1.1). r_agl lumps sensor noise and onboard-map
    // error. 100 m^2 (10 m sigma) came out best in the flight001 MC sweep (TEST-017).
    double r_agl = 100.0;
    double gate_agl_sigma = 3.0;
    double gate_agl_abs_m = 40.0;
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
    // Baro-bias random walk, m^2/s. 0.05 lets the bias move about 4 m in 5 minutes
    // (1 sigma), enough to follow a pressure change. 0.001 couldn't track drift and
    // leaked it into position (TEST-017 sweep).
    double q_baro_bias = 0.05;
    double p0_baro_bias = 25.0;
    double r_baro = 4.0;
    double gate_baro_sigma = 5.0;
    // Magnetometer (OI-003)
    double q_mag_bias = 1e-5;     // (uT/s)^2/s : mag bias random walk PSD
    double p0_mag_bias = 100.0;   // uT^2 : initial mag bias variance (sigma=10uT)
    double r_mag = 4.0;           // uT^2 : mag measurement noise (sigma=2uT)
    double gate_mag_sigma = 5.0;  // 5-sigma innovation gate for mag (Mahalanobis)
    // Heading-only fusion (R1.4) assumes a calibrated compass; this is the residual hard
    // iron after calibration, uT (1 sigma), treated as measurement noise.
    double mag_heading_cal_sigma = 0.5;
    // ApplyMagAuto (R1.4): 3-axis update only while the measured field looks like the
    // Earth field, averaged over about 1 s (time constant mag_auto_tau, in updates):
    // strength within mag_auto_norm_tol uT and dip within mag_auto_dip_tol_deg of the
    // reference. Otherwise heading only.
    double mag_auto_norm_tol = 2.0;
    double mag_auto_dip_tol_deg = 2.0;
    double mag_auto_tau = 10.0;
    // Hysteresis: after a disturbance, stay heading-only until the field has looked clean
    // for this many updates. Off by default: at these tolerances auto doesn't flap, and a
    // 30-update hold cost the Zermatt MC (99/100 and 1.77% over 3 sigma, against 100/100
    // and 0.46%) because manoeuvres trip the check briefly. With tighter tolerances noise
    // lets single 3-axis updates through a disturbance, each kicking the tilt (up to 13 deg
    // in the TEST-021 sweep); set a hold of about 30 then.
    int mag_auto_hold = 0;
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

    // Missing IMU data (samples lost upstream, a stalled driver): the nominal state was
    // carried across the hole with one sample's rate and acceleration, so it may be off
    // by more than the process noise allows. Add that uncertainty explicitly: sigma_att
    // (rad) on each attitude axis and sigma_vel (m/s) on each velocity axis. Without
    // this the gates treat the now-wrong attitude as certain and reject every
    // magnetometer correction, and a tilt error turns into runaway velocity.
    void InflateForGap(double sigma_att_rad, double sigma_vel) {
        for (int i = 0; i < 3; ++i) {
            P_(6 + i, 6 + i) += sigma_att_rad * sigma_att_rad;
            P_(3 + i, 3 + i) += sigma_vel * sigma_vel;
        }
        n_gap_++;
    }
    int gaps() const { return n_gap_; }

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
            std::abs(innov) > cfg_.gate_abs_m) { n_rej_++; n_rej_trn_++; return UpdResult::kInnovGate; }
        V19 K = (P_ * H.transpose()) / S;
        // Terrain height says nothing about the baro or mag bias. Zero their gain rows
        // before the update (R1.4), so the Joseph form below neither corrects them nor
        // shrinks their variance. Zeroing dx afterwards (as before) still contracted P.
        K(15) = 0.0; K.segment<3>(16).setZero();
        LimitGain(K, innov);
        V19 dx = K * innov;
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

    // Altitude datum: AMSL height of the local origin (p_z = 0). Set it once, for
    // example at start-up from the map height under the vehicle plus the first AGL
    // reading. Altitude AMSL is then h0 - p_z (NED, z down).
    // sigma_z: how well the datum is known (AGL noise plus map error at the start
    // point). The altitude state is re-referenced to the new datum, so its variance
    // becomes sigma_z^2, uncorrelated with the rest (keeps P positive semi-definite).
    void SetAltitudeDatum(double h0_amsl, double sigma_z = -1.0) {
        h0_ = h0_amsl; datum_set_ = true;
        if (sigma_z > 0) {
            P_.row(2).setZero(); P_.col(2).setZero();
            P_(2, 2) = sigma_z * sigma_z;
        }
    }
    bool datum_set() const { return datum_set_; }
    // Take the baro bias from a baro reading at a moment when p_z is known (normally
    // on the ground, right after SetAltitudeDatum). Covariance is left alone.
    // With sigma_baro > 0, the covariance follows the same definition: b = z_baro - z,
    // so b's covariance with every state is minus z's, plus the baro noise on b
    // itself. That's an exact linear transform, so P stays positive semi-definite.
    void InitBaroBias(double z_baro, double sigma_baro = -1.0) {
        baro_bias_ = z_baro - pos_.z();
        if (sigma_baro > 0) {
            const double pzz = P_(2, 2);
            P_.row(15) = -P_.row(2);
            P_.col(15) = -P_.col(2);
            P_(15, 15) = pzz + sigma_baro * sigma_baro;
            P_(15, 2) = P_(2, 15) = -pzz;
        }
    }
    double altitude_datum() const { return h0_; }
    double altitude_amsl() const { return h0_ - pos_.z(); }

    // Height above ground along the local vertical (tilt already removed), metres.
    // Model: agl = (h0 - p_z) - h_map(p_n, p_e) + noise.
    // This is what a real rangefinder or radar altimeter gives. Unlike ApplyTrn, the
    // vehicle's altitude is part of the prediction, so altitude error and baro bias
    // are estimated together with position instead of leaking into it. No slope gate:
    // on flat ground the update still observes altitude, it just carries no
    // horizontal information (H has no north/east part there).
    UpdResult ApplyAgl(double agl, const MapQuery& mq,
                       double* innov_out = nullptr, double* s_out = nullptr) {
        if (!aligned_ || !yaw_aligned_ || !datum_set_ || !mq.ok) return UpdResult::kSlopeGate;
        H19 H = H19::Zero();
        H(0, 0) = -mq.dn; H(0, 1) = -mq.de; H(0, 2) = -1.0;
        const double innov = agl - (h0_ - pos_.z() - mq.h);
        const double S = (H * P_ * H.transpose())(0, 0) + cfg_.r_agl;
        if (innov_out) *innov_out = innov;
        if (s_out) *s_out = S;
        if (S <= 0 || std::abs(innov) / std::sqrt(S) > cfg_.gate_agl_sigma ||
            std::abs(innov) > cfg_.gate_agl_abs_m) { n_rej_++; n_rej_agl_++; return UpdResult::kInnovGate; }
        V19 K = (P_ * H.transpose()) / S;
        // States this update must not touch get a zero gain row. The Joseph form below
        // is valid for any gain, so the covariance stays consistent with what was
        // actually applied (unlike zeroing dx after the fact).
        K.segment<3>(16).setZero();                  // mag bias
        LimitGain(K, innov);
        V19 dx = K * innov;
        x_ += dx;
        const M19 IKH = M19::Identity() - K * H;
        P_ = Repair(IKH * P_ * IKH.transpose() + cfg_.r_agl * K * K.transpose());
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
        n_upd_++; n_agl_++;
        return UpdResult::kApplied;
    }

    UpdResult ApplyZupt() {
        if (!aligned_) return UpdResult::kSlopeGate;
        
        // H observes velocity error (indices 3,4,5)
        Eigen::Matrix<double, 3, 19> H = Eigen::Matrix<double, 3, 19>::Zero();
        H.block<3,3>(0, 3) = Eigen::Matrix3d::Identity();
        
        // Innovation: we want nominal velocity to be 0
        Eigen::Vector3d y = -vel_;
        
        const double r_zupt = 0.05 * 0.05; // 0.05 m/s noise
        Eigen::Matrix3d R = r_zupt * Eigen::Matrix3d::Identity();
        Eigen::Matrix3d S = (H * P_ * H.transpose()) + R;
        
        // Very loose gate to prevent lockout if drift already happened
        double d2 = y.transpose() * S.inverse() * y;
        if (d2 > 100.0) { n_rej_++; return UpdResult::kInnovGate; }
        
        Eigen::Matrix3d S_inv = S.inverse();
        Eigen::Matrix<double, 19, 3> K = P_ * H.transpose() * S_inv;
        // Zero velocity says nothing about heading (rotating a parked vehicle about the
        // vertical leaves its velocity at zero), nor about the baro and mag biases.
        // Through the noisy-accel coupling in F, a full gain still moves them: 8 deg of
        // yaw walk in 60 s parked (TEST-018). Remove those gain components. Attitude
        // and gyro-bias errors are body-frame, so heading is the body-frame vertical
        // u = R^T e_down. The Joseph update below is valid for any gain, so the
        // covariance stays consistent with what was applied.
        {
            const V3 u = q_.conjugate() * V3::UnitZ();
            const Eigen::Matrix3d Pu = Eigen::Matrix3d::Identity() - u * u.transpose();
            K.block<3,3>(6, 0)  = Pu * K.block<3,3>(6, 0);
            K.block<3,3>(12, 0) = Pu * K.block<3,3>(12, 0);
            K.block<4,3>(15, 0).setZero();          // baro bias, mag bias
        }
        LimitGain<3>(K, y, false);
        V19 dx = K * y;

        x_ += dx;
        const M19 IKH = M19::Identity() - K * H;
        P_ = Repair(IKH * P_ * IKH.transpose() + K * R * K.transpose());
        
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
        n_zupt_++;
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
            n_rej_++; n_rej_baro_++; return UpdResult::kInnovGate;
        }
        V19 K = (P_ * H.transpose()) / S;
        K.segment<3>(16).setZero();        // baro says nothing about mag bias (R1.4)
        LimitGain(K, innov);
        V19 dx = K * innov;
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
        mag_innov_rms_ = 0.98 * mag_innov_rms_ + 0.02 * y.norm();
        
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
            n_rej_++; n_rej_mag_++; return UpdResult::kInnovGate;
        }
        
        Eigen::Matrix<double, 19, 3> K = P_ * H.transpose() * S_inv;
        LimitGain<3>(K, y, true);
        V19 dx = K * y;
        
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

    // Accelerometer reading expected at rest with the current attitude and bias:
    // R^T (-g) + b_a. Right after FeedStatic this is the parked mean exactly.
    V3 RestSpecificForce() const { return q_.conjugate() * (-g_) + ba_; }

    // Magnetometer as heading only (R1.4, OI-012). The 3-axis update (ApplyMag) explains
    // any field error with attitude, so a body-fixed disturbance (motor current, a nearby
    // steel structure) tilts roll and pitch: 2.5 deg from 6 uT on body z in TEST-021.
    // Here the measured field is levelled with the current attitude, and only the angle
    // of its horizontal part is compared with the reference:
    //   m_n = R (z - b_mag),   e = atan2(m_ref_E, m_ref_N) - atan2(m_n_E, m_n_N)
    // e is the heading error (nav frame). Error state: q_true = q * Exp(dtheta), so a
    // heading error is e3^T R dtheta. The attitude gain is projected onto the body
    // vertical, so the update can't tilt the attitude even through correlations.
    // Hard iron: a heading sees the field's direction, never its strength, so it can't
    // estimate the mag bias. Left free, the estimate ran away to 120 uT with no bias at
    // all (TEST-021 f). So this update assumes a calibrated compass: it uses the current
    // bias estimate as the calibration, never changes it, and counts the calibration's
    // residual (mag_heading_cal_sigma, uT) as noise next to the sensor noise. Noise in
    // radians = those over the horizontal field strength.
    UpdResult ApplyMagHeading(const Eigen::Vector3d& z_meas) {
        if (!aligned_ || !yaw_aligned_) return UpdResult::kSlopeGate;
        const Eigen::Matrix3d R = q_.toRotationMatrix();
        const V3 m_ref(cfg_.mag_ref_north, cfg_.mag_ref_east, cfg_.mag_ref_down);
        mag_innov_rms_ = 0.98 * mag_innov_rms_ + 0.02 * (z_meas - (R.transpose() * m_ref + mag_bias_)).norm();
        const V3 m_n = R * (z_meas - mag_bias_);
        const double h2 = m_n.head<2>().squaredNorm();
        if (h2 < 1.0) return UpdResult::kSlopeGate;            // under 1 uT horizontal: no heading
        const double innov = std::remainder(std::atan2(m_ref.y(), m_ref.x()) -
                                            std::atan2(m_n.y(), m_n.x()), 2.0 * M_PI);
        H19 H = H19::Zero();
        H.block<1, 3>(0, 6) = R.row(2);
        const double r = (cfg_.r_mag + cfg_.mag_heading_cal_sigma * cfg_.mag_heading_cal_sigma) / h2;
        const double S = (H * P_ * H.transpose())(0, 0) + r;
        if (!(S > 0.0) || std::isnan(innov) || std::abs(innov) / std::sqrt(S) > cfg_.gate_mag_sigma) {
            n_rej_++; n_rej_mag_++; return UpdResult::kInnovGate;
        }
        V19 K = (P_ * H.transpose()) / S;
        const V3 u = q_.conjugate() * V3::UnitZ();             // body-frame vertical
        K.segment<3>(6) = u * u.dot(K.segment<3>(6));          // heading only, no tilt
        K(15) = 0.0;                                           // nothing to say about baro
        K.segment<3>(16).setZero();                            // nor about the mag bias
        LimitGain(K, innov);
        const V19 dx = K * innov;
        x_ += dx;
        const M19 IKH = M19::Identity() - K * H;
        P_ = Repair(IKH * P_ * IKH.transpose() + r * K * K.transpose());
        pos_ += x_.segment<3>(0); vel_ += x_.segment<3>(3);
        const double da = x_.segment<3>(6).norm();
        if (da > 1e-9)
            q_ = (q_ * Eigen::Quaterniond(Eigen::AngleAxisd(da, x_.segment<3>(6).normalized()))).normalized();
        ba_ += x_.segment<3>(9); bg_ += x_.segment<3>(12);
        mag_bias_ += x_.segment<3>(16);
        M19 J = M19::Identity();
        J.block<3,3>(6,6) = Eigen::Matrix3d::Identity() - Skew(x_.segment<3>(6));
        P_ = Repair(J * P_ * J.transpose());
        x_.setZero();
        n_upd_++;
        return UpdResult::kApplied;
    }
    const M19& P() const { return P_; }

    // Magnetometer, chosen per sample (R1.4). The 3-axis update also observes tilt, which
    // is worth having when the field is clean (TEST-021: it carries the degraded-map
    // Monte Carlo), but any disturbance tilts the attitude (OI-012). So: 3-axis while the
    // field's strength and dip agree with the reference (each averaged, as the per-sample
    // noise alone is 2 uT and 2.4 deg), heading only when they don't, and for
    // mag_auto_hold updates after they agree again.
    UpdResult ApplyMagAuto(const Eigen::Vector3d& z_meas) {
        if (!aligned_ || !yaw_aligned_) return UpdResult::kSlopeGate;
        const V3 m_ref(cfg_.mag_ref_north, cfg_.mag_ref_east, cfg_.mag_ref_down);
        const V3 v = q_.toRotationMatrix() * (z_meas - mag_bias_);
        const double dnorm = v.norm() - m_ref.norm();
        const double ddip = std::asin(std::clamp(v.z() / std::max(v.norm(), 1e-6), -1.0, 1.0)) -
                            std::asin(m_ref.z() / m_ref.norm());
        const double a = 1.0 / std::max(1.0, cfg_.mag_auto_tau);
        mag_dnorm_ = (1 - a) * mag_dnorm_ + a * dnorm;
        mag_ddip_ = (1 - a) * mag_ddip_ + a * ddip;
        const bool clean = std::abs(mag_dnorm_) < cfg_.mag_auto_norm_tol &&
                           std::abs(mag_ddip_) < cfg_.mag_auto_dip_tol_deg * kDeg;
        mag_clean_run_ = clean ? mag_clean_run_ + 1 : 0;
        if (!clean) mag_disturbed_ = true;
        else if (mag_disturbed_ && mag_clean_run_ >= cfg_.mag_auto_hold) mag_disturbed_ = false;
        if (!mag_disturbed_) { n_mag_full_++; return ApplyMag(z_meas); }
        n_mag_head_++;
        return ApplyMagHeading(z_meas);
    }
    int mag_full_updates() const { return n_mag_full_; }
    int mag_heading_updates() const { return n_mag_head_; }

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
    Eigen::Matrix2d P_horiz() const { return P_.block<2, 2>(0, 0); }   // north/east (R1.5)
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
    int zupt() const { return n_zupt_; }
    int rej_trn() const { return n_rej_trn_; }
    int rej_agl() const { return n_rej_agl_; }
    int agl_updates() const { return n_agl_; }
    int rej_baro() const { return n_rej_baro_; }
    int rej_mag() const { return n_rej_mag_; }
    double mag_innov_rms() const { return mag_innov_rms_; }
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
    // Per-update correction limits (R1.4). Old approach: clamp dx after computing it,
    // then update P with the full gain, so P shrank as if the whole correction had been
    // applied (one clamped ZUPT left 4.5 m/s of error behind a 0.087 m/s sigma). Here the
    // gain rows of a block are scaled down until its correction fits the limit. The
    // Joseph update is valid for any gain, so P matches what was actually applied.
    template <int M>
    void LimitGain(Eigen::Matrix<double, 19, M>& K, const Eigen::Matrix<double, M, 1>& y,
                   bool with_mag_bias) {
        const struct { int i; double lim; } blocks[] = {
            {3, cfg_.clamp_dv}, {6, cfg_.clamp_dtheta_deg * kDeg}, {9, cfg_.clamp_dva},
            {12, cfg_.clamp_dvg_deg * kDeg}, {16, with_mag_bias ? cfg_.clamp_dvm : -1.0}};
        for (const auto& b : blocks) {
            if (b.lim <= 0.0) continue;
            const double n = (K.template block<3, M>(b.i, 0) * y).norm();
            if (n > b.lim) { K.template block<3, M>(b.i, 0) *= b.lim / n; n_clamp_++; }
        }
    }
    void LimitGain(V19& K, double y) {
        Eigen::Matrix<double, 1, 1> yy; yy(0) = y;
        LimitGain<1>(K, yy, false);
    }
public:
    int clamps() const { return n_clamp_; }   // gain-limit events (R1.4)
private:
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
    int n_zupt_ = 0;
    int n_rej_trn_ = 0, n_rej_baro_ = 0, n_rej_mag_ = 0;
    int n_rej_agl_ = 0, n_agl_ = 0;
    int n_gap_ = 0;
    int n_clamp_ = 0;
    int n_mag_full_ = 0, n_mag_head_ = 0;
    double mag_dnorm_ = 0.0, mag_ddip_ = 0.0;
    int mag_clean_run_ = 0;
    bool mag_disturbed_ = false;
    double h0_ = 0.0;
    bool datum_set_ = false;
    double mag_innov_rms_ = 0.0;
    bool aligned_ = false, yaw_aligned_ = false;
};

}  // namespace gdn
