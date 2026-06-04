#include "inekf_mini/robot_state.h"
#include "inekf_mini/correction.h"
#include "inekf_mini/math/lie_group.hpp"

namespace {
Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m(0,0)=0;      m(0,1)=-v(2);  m(0,2)=v(1);
    m(1,0)=v(2);   m(1,1)=0;      m(1,2)=-v(0);
    m(2,0)=-v(1);  m(2,1)=v(0);   m(2,2)=0;
    return m;
}
}

namespace inekf {
// 預設初始化
RobotState::RobotState(const RobotStateConfig& config, FrameType f_type, ErrorType e_type) : 
    config_(config),
    frame_internal(f_type), error_internal(e_type),  frame_type(frame_internal), error_type(error_internal) 
    {     
    R_imu_in_base = config_.R_imu_in_base;
    p_imu_in_base = config_.p_imu_in_base;

    // initial state SEk3
    BASE_DIM_X = 5;
    X_ = Eigen::MatrixXd::Identity(BASE_DIM_X, BASE_DIM_X); // R,v,p

    DIM_THETA = bias_estimate_enabled() ? 6 : 0;
    Theta_ = Eigen::VectorXd::Zero(DIM_THETA);

    BASE_DIM_P = DIM_THETA + 3*(BASE_DIM_X - 2);
    P_ = Eigen::MatrixXd::Zero(BASE_DIM_P, BASE_DIM_P);
    Q_ = Eigen::MatrixXd::Zero(BASE_DIM_P, BASE_DIM_P);

    if (bias_estimate_enabled()) {
        P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * std::pow(config_.bg_init_std, 2);
        P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * std::pow(config_.ba_init_std, 2);

        Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * std::pow(config_.gyro_bias_std, 2);
        Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * std::pow(config_.accel_bias_std, 2);
    }
    P_.block<3, 3>(DIM_THETA, DIM_THETA)         = Eigen::Matrix3d::Identity() * std::pow(config_.rot_init_std, 2);
    P_.block<3, 3>(DIM_THETA + 3, DIM_THETA + 3) = Eigen::Matrix3d::Identity() * std::pow(config_.vel_init_std, 2);
    P_.block<3, 3>(DIM_THETA + 6, DIM_THETA + 6) = Eigen::Matrix3d::Identity() * std::pow(config_.pos_init_std, 2);

    Q_.block<3, 3>(DIM_THETA, DIM_THETA)         = Eigen::Matrix3d::Identity() * std::pow(config_.gyro_noise_std, 2);
    Q_.block<3, 3>(DIM_THETA + 3, DIM_THETA + 3) = Eigen::Matrix3d::Identity() * std::pow(config_.accel_noise_std, 2);
}

void RobotState::reset(const Eigen::Vector3d& mean_gyro, const Eigen::Vector3d& mean_acc,
                       const Eigen::Vector3d& var_gyro, const Eigen::Vector3d& var_acc) {
    X_ = Eigen::MatrixXd::Identity(BASE_DIM_X, BASE_DIM_X);
    // posture reset
    Eigen::Vector3d unit_gravity_body = mean_acc.normalized();
    Eigen::Vector3d unit_gravity_world(0.0, 0.0, 1.0);
    Eigen::Quaterniond q0;
    if (unit_gravity_body.dot(unit_gravity_world) < -0.9999) { 
        // singular case, choose X-axis
        q0 = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    } else { 
        // shortest rotation from body gravity to world gravity
        q0 = Eigen::Quaterniond::FromTwoVectors(unit_gravity_body, unit_gravity_world);
    }
    set_rotation(q0.toRotationMatrix());
    // bias reset
    if (bias_estimate_enabled()) {
        Theta_.segment<3>(0) = mean_gyro;
        Theta_.segment<3>(3) = mean_acc - get_rotation().transpose()*g;
    }
    Q_.block<3, 3>(DIM_THETA, DIM_THETA) = var_gyro.asDiagonal();
    Q_.block<3, 3>(DIM_THETA+ 3, DIM_THETA + 3) = var_acc.asDiagonal();
}

// d type 的管理函式，根據修正塊的觀測結果調整矩陣結構
void RobotState::update_structure(const std::vector<std::shared_ptr<DynamicCorrectionBlock>>& blocks) {
    int Xobs_num = dim_X() - BASE_DIM_X;
    std::vector<int*> Xobs_ptrs(Xobs_num, nullptr);
    std::vector<std::pair<DynamicCorrectionBlock::Observation*, std::shared_ptr<DynamicCorrectionBlock>>> new_obs;

    // -------------------------------------------------------------------------
    // 步驟 1：掃描觀測狀態並分類（Status Scanning & Classification）
    // -------------------------------------------------------------------------
    int active_obs_num = 0;
    for (const auto& block : blocks) {
        for (auto& ob : block->obs) {
            if (ob.is_active) {
                if (*(ob.index_ptr) != -1) { // in_matrix
                    int ob_in_Xobs_idx = *(ob.index_ptr) - BASE_DIM_X; 
                    Xobs_ptrs[ob_in_Xobs_idx] = ob.index_ptr.get();
                    active_obs_num++;
                } else {
                    new_obs.push_back({&ob, block});
                }
            } else {
                *(ob.index_ptr) = -1; 
            }
        }
    }

    // -------------------------------------------------------------------------
    // 步驟 2：雙指標原地壓縮（In-place Contraction）
    // -------------------------------------------------------------------------
    int left = 0;
    int right = Xobs_num - 1;
    while (true) {   
        while (left < right && Xobs_ptrs[left] != nullptr) left++; // left 往右走，直到踩到「坑」（nullptr）
        while (left < right && Xobs_ptrs[right] == nullptr) right--; // right 往左走，直到踩到「活人」（有效指標）
        if (left >= right) break;

        int X_target_idx = left  + BASE_DIM_X;  int X_source_idx = right + BASE_DIM_X;
        int P_target_idx = 3*left  + BASE_DIM_P;  int P_source_idx = 3*right + BASE_DIM_P;
        
        X_.block<3,1>(0, X_target_idx) = X_.block<3,1>(0, X_source_idx);
        Eigen::Matrix3d source_diagonal = P_.block<3, 3>(P_source_idx, P_source_idx);
        P_.block(0, P_target_idx, dim_P(), 3) = P_.block(0, P_source_idx, dim_P(), 3);
        P_.block(P_target_idx, 0, 3, dim_P()) = P_.block(0, P_source_idx, dim_P(), 3).transpose();
        P_.block<3, 3>(P_target_idx, P_target_idx) = source_diagonal;
        Q_.block<3, 3>(P_target_idx, P_target_idx) = Q_.block<3, 3>(P_source_idx, P_source_idx);

        // Xobs_ptrs[left] 已為 -1
        *(Xobs_ptrs[right]) = X_target_idx;
        Xobs_ptrs[left] = Xobs_ptrs[right];
        Xobs_ptrs[right] = nullptr;

        left++;
        right--;
    }

    // -------------------------------------------------------------------------
    // 步驟 3：新住客填補或擴張（State Augmentation & Initialization）
    // -------------------------------------------------------------------------
    int now_active_X_dim = BASE_DIM_X + active_obs_num;
    int now_active_P_dim = BASE_DIM_P + 3*active_obs_num;

    int new_obs_size = new_obs.size();
    int required_X_dim = now_active_X_dim + new_obs_size;
    int required_P_dim = now_active_P_dim + 3*new_obs_size;

    if (required_X_dim != dim_X()){
        X_.conservativeResize(required_X_dim, required_X_dim);
        P_.conservativeResize(required_P_dim, required_P_dim);
        Q_.conservativeResize(required_P_dim, required_P_dim);
    }
    if (new_obs_size > 0) {
        X_.block(0, now_active_X_dim, required_X_dim, new_obs_size).setZero();
        X_.block(now_active_X_dim, 0, new_obs_size, required_X_dim).setZero();
        X_.block(now_active_X_dim, now_active_X_dim, new_obs_size, new_obs_size).setIdentity();
        Q_.block(0,now_active_P_dim, required_P_dim, 3*new_obs_size).setZero();
        Q_.block(now_active_P_dim, 0, 3*new_obs_size, required_P_dim).setZero();
    }

    // 把新住客依序填入尾端
    for (const auto& new_ob : new_obs) {
        const auto& ob = new_ob.first;
        const auto& block = new_ob.second;
        *(ob->index_ptr) = now_active_X_dim;
        // =========================================================================================================

        Eigen::Matrix3d R = get_rotation();
        Eigen::Vector3d v = get_velocity();
        Eigen::Vector3d p = get_position();

        bool is_left = (block->frame_type == frame_type);
        // 決定 Invariant 幾何算子
        Eigen::Vector3d spatial_diff = is_left ? (ob->data - R * ob->outer_ref) : (ob->outer_ref - R * ob->data);
        Eigen::Matrix3d skew_v = block->b[0] * skew(ob->outer_ref);
        Eigen::Matrix3d trans_noise = Eigen::Matrix3d::Zero();
        if (is_left && error_type == ErrorType::LeftInvariant){ trans_noise = R.transpose() * ob->noise * R; }
        else if (!is_left && error_type == ErrorType::RightInvariant){ trans_noise = R * ob->noise * R.transpose(); }
        else { // 觀測與系統的 error type 不相同
            skew_v = skew(ob->data);
            trans_noise = ob->noise;
        }
        
        // 新 ob 的初始化
        X_.block<3,1>(0, now_active_X_dim) = (spatial_diff - block->b[1] * v - block->b[2] * p) / block->b[3];

        Eigen::MatrixXd P_R_cols = P_.block(0, P_Xidx(0), now_active_P_dim, 3);
        Eigen::MatrixXd P_v_cols = P_.block(0, P_Xidx(3), now_active_P_dim, 3);
        Eigen::MatrixXd P_p_cols = P_.block(0, P_Xidx(6), now_active_P_dim, 3);
        double inv_b3_sq = 1.0 / (block->b[3] * block->b[3]);

        Eigen::MatrixXd new_col = (P_R_cols * skew_v + P_v_cols * block->b[1] + P_p_cols * block->b[2]) * inv_b3_sq;
        P_.block(0, now_active_P_dim, now_active_P_dim, 3) = new_col;
        P_.block(now_active_P_dim, 0, 3, now_active_P_dim) = new_col.transpose();
        Eigen::Matrix3d P_new_self = -skew_v * new_col.block<3,3>(P_Xidx(0),0)
         + block->b[1] * new_col.block<3,3>(P_Xidx(3),0)
         + block->b[2] * new_col.block<3,3>(P_Xidx(6),0);

        P_.block<3, 3>(now_active_P_dim, now_active_P_dim) = P_new_self + trans_noise * inv_b3_sq;
        Q_.block<3, 3>(now_active_P_dim, now_active_P_dim) = block->process_noise;

        // =========================================================================================================
        now_active_X_dim++;
        now_active_P_dim += 3;
    }
}

Eigen::MatrixXd& RobotState::covariance_processing(Eigen::MatrixXd& P_ref, FrameType block_frame_type, bool is_pre) {
    const bool is_left = (block_frame_type == frame_type);
    const bool cond1 = (is_left && error_type == ErrorType::RightInvariant);
    const bool cond2 = (!is_left && error_type == ErrorType::LeftInvariant);
    if (cond1 || cond2) {
        const int dim_xi = dim_P() - DIM_THETA;
        const Eigen::MatrixXd Adj = cond1 ? (is_pre ? SEk3::adjoint_inv(X_) : SEk3::adjoint(X_))
                                          : (is_pre ? SEk3::adjoint(X_) : SEk3::adjoint_inv(X_));                                
        auto P_block = P_ref.block(DIM_THETA, DIM_THETA, dim_xi, dim_xi);
        P_block = Adj * P_block * Adj.transpose(); // 直接改變 P_ref 的對應區塊，避免額外的記憶體複製
    }
    return P_ref;
}
} // namespace inekf