#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include "inekf_mini/types.h"

namespace inekf {
class DynamicCorrectionBlock;

struct RobotStateConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 確保 Eigen 記憶體對齊安全

    // --- 模式設定 ---
    bool bias_estimate_enable = true;

    // --- 幾何外參 (IMU 相對於 Base) ---
    Eigen::Matrix3d R_imu_in_base = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_imu_in_base = Eigen::Vector3d::Zero();
    
    // --- 初始狀態不確定性 (P 矩陣初始化用) ---
    double rot_init_std = 1e-2;
    double vel_init_std = 1e-1;
    double pos_init_std = 1e-2;
    double bg_init_std  = 1e-3;
    double ba_init_std  = 1e-2;

    // --- 過程噪聲密度 (Q 矩陣傳播用) ---
    // 這是解決漂移的關鍵，數值通常比 init_std 小很多
    double gyro_noise_std    = 1e-4; // 陀螺儀白噪聲
    double accel_noise_std   = 1e-3; // 加速度計白噪聲
    double gyro_bias_std     = 1e-6; // 陀螺儀隨機遊走 (Bias 穩定性)
    double accel_bias_std    = 1e-5; // 加速度計隨機遊走
};

class RobotState {
private:
    // Internal storage for frame and error types, with public read-only access
    FrameType frame_internal;
    ErrorType error_internal;

    RobotStateConfig config_;

    Eigen::MatrixXd X_;      // SEn(3) state matrix (R, v, p, + extra states)
    Eigen::VectorXd Theta_;  // 6x1 Bias (bg, ba)
    Eigen::MatrixXd P_;      // covariance matrix
    Eigen::MatrixXd Q_;      // Process noise covariance

    // base dimensions for indexing
    int BASE_DIM_X;
    int DIM_THETA;
    int BASE_DIM_P;
    Eigen::Vector3d g = Eigen::Vector3d(0, 0, -9.80665);

    // IMU 在 Base 框架下的外參，這是為了方便計算 IMU 觀測對 Base 狀態的影響
    Eigen::Matrix3d R_imu_in_base;
    Eigen::Vector3d p_imu_in_base;
public:
    // read-only access
    const FrameType& frame_type;
    const ErrorType& error_type;
    void setType(FrameType f, ErrorType e) { frame_internal = f; error_internal = e; }
    bool bias_estimate_enabled() const { return config_.bias_estimate_enable; }

    RobotState(const RobotStateConfig& config, FrameType f_type, ErrorType e_type);

    // functions
    void reset(const Eigen::Vector3d& bg0, const Eigen::Vector3d& ba0,
               const Eigen::Vector3d& vg0, const Eigen::Vector3d& va0);
    void propagate(const Eigen::Vector3d& w_raw, const Eigen::Vector3d& a_raw, double dt);
    void update_structure(const std::vector<std::shared_ptr<DynamicCorrectionBlock>>& blocks); // for d type correction
    Eigen::MatrixXd& covariance_processing(Eigen::MatrixXd& P, FrameType block_frame_type, bool is_pre);

    // dimensions and indices
    int dim_theta() const { return Theta_.size(); } // 6
    int dim_X() const { return X_.cols(); } // n + 3
    int dim_P() const { return P_.rows(); } // 3*(n+1) + 6
    int P_Xidx(int offset) const { return DIM_THETA + offset; } // 基礎狀態在 P 中的起始索引
    int P_Xvecidx(int col_idx) const { return BASE_DIM_P + 3*(col_idx - BASE_DIM_X); }
    
    // Getters
    Eigen::Matrix3d get_rotation() const { return X_.block<3, 3>(0, 0); }
    Eigen::Vector3d get_velocity() const { return X_.block<3, 1>(0, 3); }
    Eigen::Vector3d get_position() const { return X_.block<3, 1>(0, 4); }
    Eigen::Vector3d get_vector(int col_idx) const { return X_.block<3, 1>(0, col_idx); }
    Eigen::Vector3d get_bg() const { return Theta_.head<3>(); }
    Eigen::Vector3d get_ba() const { return Theta_.tail<3>(); }

    const Eigen::VectorXd& get_theta() const { return Theta_; }
    const Eigen::MatrixXd& get_X() const { return X_; }
    const Eigen::MatrixXd& get_P() const { return P_; }
    const Eigen::MatrixXd& get_Q() const { return Q_; }

    // Setters
    void set_rotation(const Eigen::Matrix3d R) { X_.block<3, 3>(0, 0) = R;}
    void set_velocity(const Eigen::Vector3d v) { X_.block<3, 1>(0, 3) = v;}
    void set_position(const Eigen::Vector3d p) { X_.block<3, 1>(0, 4) = p; }
    void set_vector(int col_idx, const Eigen::Vector3d vec) { X_.block<3, 1>(0, col_idx) = vec; }
    
    void set_theta(const Eigen::VectorXd& theta) { Theta_ = theta; }
    void set_X(const Eigen::MatrixXd& X) { X_ = X; }
    void set_P(const Eigen::MatrixXd& P) { P_ = P; }
    void set_Q_imu(const Eigen::Vector3d gyro_var, const Eigen::Vector3d accel_var) { 
        Q_.block<3,3>(DIM_THETA, DIM_THETA) = gyro_var.asDiagonal();
        Q_.block<3,3>(DIM_THETA + 3, DIM_THETA + 3) = accel_var.asDiagonal();
    }

    // --- Isaac Lab Standard Observation Mapping ---
    Eigen::Vector3d world_lin_vel() const {
        if (frame_type == FrameType::World) { return get_velocity(); }
        return -get_rotation().transpose() * get_velocity();
    }

    Eigen::Vector3d base_lin_vel() const {
        if (frame_type == FrameType::Body) { return -get_velocity(); }
        return get_rotation().transpose() * get_velocity();
    }

    Eigen::Vector3d base_ang_vel(const Eigen::Vector3d& w_raw) const {
        return R_imu_in_base * (w_raw - get_bg());
    }
    
    Eigen::Vector3d projected_gravity() const {
        if (frame_type == FrameType::World) { 
            return -get_rotation().row(2).transpose(); 
        }
        return -get_rotation().col(2);
    }
};
} // namespace inekf

#endif