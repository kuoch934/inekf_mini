#ifndef WELFORD_HPP
#define WELFORD_HPP

#include <Eigen/Dense>

namespace inekf {
template <int Dim = 3>
class Welford {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Welford() 
        : n_(0), 
          M1(Eigen::Matrix<double, Dim, 1>::Zero()), 
          S2(Eigen::Matrix<double, Dim, 1>::Zero()),
          M2(Eigen::Matrix<double, Dim, 1>::Zero()) {}

    void reset() {
        n_ = 0;
        M1.setZero();
        S2.setZero();
        M2.setZero();
    }

    void update(const Eigen::Matrix<double, Dim, 1>& x) {
        n_++;
        Eigen::Matrix<double, Dim, 1> delta = x - M1;
        M1 += delta / n_;
        S2 += delta.array() * (x.array() - M1.array());
        M2 = S2 / static_cast<double>(n_);
    }

    void update_weighted(const Eigen::Matrix<double, Dim, 1>& x, double alpha) {
        Eigen::Matrix<double, Dim, 1> delta = x - M1;
        M1 += alpha * delta;
        M2.array() =  (1 - alpha) * (M2.array() + alpha * delta.array() * delta.array());
    }

    int count() const { return n_; }
    Eigen::Matrix<double, Dim, 1> mean() const { return M1; }
    Eigen::Matrix<double, Dim, 1> variance() const { return M2; }
    Eigen::Matrix<double, Dim, 1> stddev() const { return M2.cwiseMax(0.0).array().sqrt(); }
private:
    int n_;
    Eigen::Matrix<double, Dim, 1> M1; // mean
    Eigen::Matrix<double, Dim, 1> S2; // sum of squared differences (Standard mode)
    Eigen::Matrix<double, Dim, 1> M2; // variance
};
} // namespace inekf

#endif // WELFORD_HPP