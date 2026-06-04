#ifndef LIE_GROUP_HPP
#define LIE_GROUP_HPP

#include <Eigen/Dense>

namespace inekf {
Eigen::Matrix3d GammaSO3(const Eigen::Vector3d& w, int m);

namespace SO3 {
    inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
        Eigen::Matrix3d m;
        m(0,0)=0;      m(0,1)=-v(2);  m(0,2)=v(1);
        m(1,0)=v(2);   m(1,1)=0;      m(1,2)=-v(0);
        m(2,0)=-v(1);  m(2,1)=v(0);   m(2,2)=0;
        return m;
    }
    inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
        return hat(v);
    }
    inline Eigen::Matrix3d Exp(const Eigen::Vector3d& w) { return GammaSO3(w, 0); }
    inline Eigen::Matrix3d leftJacobian(const Eigen::Vector3d& w) { return GammaSO3(w, 1); }
    inline Eigen::Matrix3d rightJacobian(const Eigen::Vector3d& w) { return GammaSO3(-w, 1); }
}

namespace SEk3{
    inline Eigen::MatrixXd inverse(const Eigen::MatrixXd& X) {
        int k = X.cols() - 3;
        Eigen::MatrixXd invX = Eigen::MatrixXd::Identity(3 + k, 3 + k);
        Eigen::Matrix3d Rt = X.block<3, 3>(0, 0).transpose();
        invX.block<3, 3>(0, 0) = Rt;
        if (k > 0) {
            invX.block(0, 3, 3, k).noalias() = -Rt * X.block(0, 3, 3, k);
        }
        return invX;
    }

    inline Eigen::MatrixXd adjoint(const Eigen::MatrixXd& X) {
        int dim_xi = 3 * (X.cols() - 2);
        Eigen::MatrixXd AdjX = Eigen::MatrixXd::Zero(dim_xi, dim_xi);
        Eigen::Matrix3d R = X.block<3, 3>(0, 0);
        AdjX.block<3, 3>(0, 0) = R;
        for (int i = 1; i <= X.cols()-3; ++i) {
            AdjX.block<3, 3>(3 * i, 3 * i) = R;
            AdjX.block<3, 3>(3 * i, 0).noalias() = SO3::hat(X.block<3, 1>(0, i + 2)) * R;
        }
        return AdjX;
    }

    inline Eigen::MatrixXd adjoint_inv(const Eigen::MatrixXd& X) {
        int dim_xi = 3 * (X.cols() - 2);
        Eigen::MatrixXd AdjX = Eigen::MatrixXd::Zero(dim_xi, dim_xi);
        Eigen::Matrix3d Rt = X.block<3, 3>(0, 0).transpose();
        AdjX.block<3, 3>(0, 0) = Rt;
        for (int i = 1; i <= X.cols()-3; ++i) {
            AdjX.block<3, 3>(3 * i, 3 * i) = Rt;
            AdjX.block<3, 3>(3 * i, 0).noalias() = -Rt * SO3::hat(X.block<3, 1>(0, i + 2));
        }
        return AdjX;
    }

    inline Eigen::MatrixXd Exp(const Eigen::VectorXd& xi) {
        assert(xi.size() >= 3 && xi.size() % 3 == 0);

        int k = xi.size() / 3 - 1;
        Eigen::MatrixXd X = Eigen::MatrixXd::Identity(3 + k, 3 + k);

        Eigen::Vector3d phi = xi.head<3>();
        X.block<3, 3>(0, 0) = SO3::Exp(phi);
        Eigen::Matrix3d lJ = SO3::leftJacobian(phi);
        if (k > 0) {
            Eigen::Map<const Eigen::MatrixXd> V(xi.data() + 3, 3, k);
            X.block(0, 3, 3, k).noalias() = lJ * V;
        }
        return X;
    }
}
} // namespace inekf

#endif // LIE_GROUP_HPP
