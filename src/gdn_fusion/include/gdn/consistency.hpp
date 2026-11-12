#pragma once
// R1.5: filter consistency statistics. ROS-free; used by mc_campaign and later by the
// SITL scorer (R2.7). A filter is consistent when its errors match the covariance it
// reports. An overconfident filter (small sigma, large error) is the failure that
// matters most for integrity (REQ-INT-001): it is wrong and says it is right.
//
// Horizontal position, per sample:
//   NEES = e^T P_h^-1 e, with e the 2D error and P_h the north/east covariance block.
//     Chi-square with 2 degrees of freedom if consistent: mean 2, and
//     P(NEES > 9) = exp(-4.5) = 1.11% (outside the 3-sigma ellipse).
//   Over 3 sigma (REQ-INT-001): |e| > 3 * sigma_pos, where sigma_pos =
//     sqrt(P_nn + P_ee) is what the node publishes on /gdn/sigma. This is looser than
//     the ellipse (for a round P, sigma_pos = sqrt(2) sigma, so the radius is 4.24
//     sigma and the expected rate exp(-9) = 0.012%), which is why the requirement can
//     ask for at most 1%.
// Scalar updates (TRN, AGL), per update:
//   NIS = innov^2 / S. Chi-square with 1 degree of freedom: mean 1, and
//   P(NIS > 9) = 0.27%. Computed before the innovation gate, so rejected updates count.
//
// Samples along one trajectory are strongly correlated in time, so these are
// descriptive statistics, not exact chi-square tests over N independent draws. A few
// false-lock seeds put NEES in the thousands and dominate the mean, so the median is
// reported too (chi-square medians: 1.386 for 2 dof, 0.455 for 1 dof).
#include <Eigen/Dense>
#include <algorithm>
#include <cstddef>
#include <vector>

namespace gdn {

class ConsistencyStats {
public:
    void AddPosition(const Eigen::Vector2d& err, const Eigen::Matrix2d& P_h, double sigma_pub) {
        Eigen::LDLT<Eigen::Matrix2d> ldlt(P_h);
        double nees;
        if (ldlt.info() == Eigen::Success && ldlt.isPositive() && P_h.determinant() > 0.0)
            nees = err.dot(ldlt.solve(err));
        else
            nees = err.squaredNorm() > 0.0 ? 1e12 : 0.0;    // collapsed covariance
        n_pos_++;
        nees_sum_ += nees; nees_.push_back(nees);
        if (nees > 9.0) n_nees_over9_++;
        if (err.norm() > 3.0 * sigma_pub) n_over3s_++;
    }
    void AddInnovation(double innov, double S) {
        const double nis = S > 0.0 ? innov * innov / S : 1e12;
        n_innov_++;
        nis_sum_ += nis; nis_.push_back(nis);
        if (nis > 9.0) n_nis_over9_++;
    }
    void Merge(const ConsistencyStats& o) {
        n_pos_ += o.n_pos_; n_innov_ += o.n_innov_;
        nees_sum_ += o.nees_sum_; nis_sum_ += o.nis_sum_;
        n_nees_over9_ += o.n_nees_over9_; n_over3s_ += o.n_over3s_; n_nis_over9_ += o.n_nis_over9_;
        nees_.insert(nees_.end(), o.nees_.begin(), o.nees_.end());
        nis_.insert(nis_.end(), o.nis_.begin(), o.nis_.end());
    }

    size_t positions() const { return n_pos_; }
    size_t innovations() const { return n_innov_; }
    double nees_mean() const { return n_pos_ ? nees_sum_ / static_cast<double>(n_pos_) : 0.0; }  // expect 2
    double nees_over9_pct() const { return Pct(n_nees_over9_, n_pos_); }         // expect 1.11
    double over3sigma_pct() const { return Pct(n_over3s_, n_pos_); }             // REQ-INT-001: <= 1
    double nis_mean() const { return n_innov_ ? nis_sum_ / static_cast<double>(n_innov_) : 0.0; }  // expect 1
    double nis_over9_pct() const { return Pct(n_nis_over9_, n_innov_); }         // expect 0.27
    double nees_median() const { return Median(nees_); }                         // expect 1.386
    double nis_median() const { return Median(nis_); }                           // expect 0.455

private:
    static double Pct(size_t k, size_t n) { return n ? 100.0 * static_cast<double>(k) / static_cast<double>(n) : 0.0; }
    static double Median(std::vector<double> v) {
        if (v.empty()) return 0.0;
        const size_t m = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(m), v.end());
        return v[m];
    }
    std::vector<double> nees_, nis_;
    size_t n_pos_ = 0, n_innov_ = 0, n_nees_over9_ = 0, n_over3s_ = 0, n_nis_over9_ = 0;
    double nees_sum_ = 0.0, nis_sum_ = 0.0;
};

}  // namespace gdn
