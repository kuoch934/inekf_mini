
#include "inekf_mini/robot_state.h"
#include "inekf_mini/correction.h"
#include "inekf_mini/math/lie_group.hpp"

namespace inekf {
void CorrectionBlock::correct(RobotState& state) {
    int obs_num = obs.size();
    bool is_left = (frame_type == state.frame_type);
    double sign = is_left ? 1.0 : -1.0; // is right (-H)

    Eigen::MatrixXd X = state.get_X();
    Eigen::Matrix3d R = state.get_rotation(); Eigen::Matrix3d Rt = R.transpose();
    Eigen::MatrixXd v = state.get_velocity();
    Eigen::MatrixXd p = state.get_position();

    Eigen::MatrixXd P = state.get_P();
    P = state.covariance_processing(P, frame_type, true);

    Eigen::VectorXd  Z  = Eigen::VectorXd::Zero(3*obs_num);
    Eigen::MatrixXd PHT = Eigen::MatrixXd::Zero(state.dim_P(), 3*obs_num);
    Eigen::MatrixXd  S  = Eigen::MatrixXd::Zero(3*obs_num, 3*obs_num);
    int i = 0;
    for (auto& ob : obs) {
        // P*HT
        auto P_R_col = P.block(0, state.P_Xidx(0), state.dim_P(), 3);
        auto P_v_col = P.block(0, state.P_Xidx(3), state.dim_P(), 3);
        auto P_p_col = P.block(0, state.P_Xidx(6), state.dim_P(), 3);
        Eigen::Matrix3d skew_ref = SO3::hat(ob.outer_ref);

        PHT.block(0, 3*i, state.dim_P(), 3) = sign * ( b[0]*P_R_col*skew_ref + b[1]*P_v_col + b[2]*P_p_col);
        // S = H*P*HT + N_bar
        auto PHT_R = PHT.block(state.P_Xidx(0), 0, 3, 3*(i+1));
        auto PHT_v = PHT.block(state.P_Xidx(3), 0, 3, 3*(i+1));
        auto PHT_p = PHT.block(state.P_Xidx(6), 0, 3, 3*(i+1));

        S.block(3*i,0,3,3*(i+1)) = sign * ( b[0]*skew_ref.transpose()*PHT_R + b[1]*PHT_v + b[2]*PHT_p);
        S.block(0,3*i,3*i,3) = S.block(3*i,0,3,3*i).transpose();
        // Z
        Eigen::Vector3d y_estimated = b[1] * v + b[2] * p;
        if (is_left) {
            Z.segment<3>(3*i) = Rt * (ob.data - y_estimated) - b[0]*ob.outer_ref;
            S.block<3,3>(3*i,3*i) += Rt * ob.noise * R;
        } else {
            Z.segment<3>(3*i) = R * ob.data + y_estimated - b[0]*ob.outer_ref;
            S.block<3,3>(3*i,3*i) += R * ob.noise * Rt;
        }
        i++;
    }
    Eigen::MatrixXd K = S.ldlt().solve(PHT.transpose()).transpose();
    Eigen::MatrixXd KPHT = K * PHT.transpose();
    Eigen::MatrixXd P_new = P - KPHT - KPHT.transpose() + K * S * K.transpose();

    Eigen::VectorXd delta = K * Z;
    Eigen::MatrixXd dX;
    if (state.bias_estimate_enabled()){ 
        dX = SEk3::Exp(delta.tail(state.dim_P() - state.dim_theta()));
        state.set_theta(state.get_theta() + delta.head(state.dim_theta()));
    } else {
        dX = SEk3::Exp(delta);
    }
    X = is_left? X * dX : dX * X;
    state.set_X(X);
    state.set_P(state.covariance_processing(P_new, frame_type, false));
}

void DynamicCorrectionBlock::correct(RobotState& state) { 
    int block_obs_in_X = 0;
    bool need_structure_update = false;
    for (auto& ob : obs) {
        bool in_matrix = (*(ob.index_ptr) != -1);
        if (in_matrix) { block_obs_in_X++; } // 包含false但還在矩陣裡的觀測(利用最後一幀)
        if ((in_matrix != ob.is_active) && !need_structure_update) { need_structure_update = true; }
    }

    if (block_obs_in_X > 0){
        bool is_left = (frame_type == state.frame_type);
        double sign = is_left ? 1.0 : -1.0; // is right (-H)

        Eigen::MatrixXd X = state.get_X();
        Eigen::Matrix3d R = state.get_rotation(); Eigen::Matrix3d Rt = R.transpose();
        Eigen::MatrixXd v = state.get_velocity();
        Eigen::MatrixXd p = state.get_position();

        Eigen::MatrixXd P = state.get_P();
        P = state.covariance_processing(P, frame_type, true);

        Eigen::VectorXd  Z  = Eigen::VectorXd::Zero(3*block_obs_in_X);
        Eigen::MatrixXd PHT = Eigen::MatrixXd::Zero(state.dim_P(), 3*block_obs_in_X);
        Eigen::MatrixXd  S  = Eigen::MatrixXd::Zero(3*block_obs_in_X, 3*block_obs_in_X);
        int i = 0;
        for (auto& ob : obs) {
            if (*(ob.index_ptr) != -1) { // in matrix
                // P*HT
                auto P_R_col = P.block(0, state.P_Xidx(0), state.dim_P(), 3);
                auto P_v_col = P.block(0, state.P_Xidx(3), state.dim_P(), 3);
                auto P_p_col = P.block(0, state.P_Xidx(6), state.dim_P(), 3);
                auto P_d_col = P.block(0, state.P_Xvecidx(*(ob.index_ptr)), state.dim_P(), 3);
                Eigen::Matrix3d skew_ref = SO3::hat(ob.outer_ref);

                PHT.block(0, 3*i, state.dim_P(), 3) = sign * ( b[0]*P_R_col*skew_ref + b[1]*P_v_col + b[2]*P_p_col + b[3]*P_d_col);
                // S = H*P*HT + N_bar
                auto PHT_R = PHT.block(state.P_Xidx(0), 0, 3, 3*(i+1));
                auto PHT_v = PHT.block(state.P_Xidx(3), 0, 3, 3*(i+1));
                auto PHT_p = PHT.block(state.P_Xidx(6), 0, 3, 3*(i+1));
                auto PHT_d = PHT.block(state.P_Xvecidx(*(ob.index_ptr)), 0, 3, 3*(i+1));

                S.block(3*i,0,3,3*(i+1)) = sign * ( b[0]*skew_ref.transpose()*PHT_R + b[1]*PHT_v + b[2]*PHT_p + b[3]*PHT_d);
                S.block(0,3*i,3*i,3) = S.block(3*i,0,3,3*i).transpose();
                // Z
                Eigen::Vector3d y_estimated = b[1] * v + b[2] * p + b[3] * state.get_vector(*(ob.index_ptr));
                if (is_left) {
                    Z.segment<3>(3*i) = Rt * (ob.data - y_estimated) - b[0]*ob.outer_ref;
                    S.block<3,3>(3*i,3*i) += Rt * ob.noise * R;
                } else {
                    Z.segment<3>(3*i) = R * ob.data + y_estimated - b[0]*ob.outer_ref;
                    S.block<3,3>(3*i,3*i) += R * ob.noise * Rt;
                }
                i++;
            }
        }
        Eigen::MatrixXd K = S.ldlt().solve(PHT.transpose()).transpose();
        Eigen::MatrixXd KHP = K * PHT.transpose();
        Eigen::MatrixXd P_new = P - KHP - KHP.transpose() + K * S * K.transpose();

        Eigen::VectorXd delta = K * Z;
        Eigen::MatrixXd dX;
        if (state.bias_estimate_enabled()) {
            dX = SEk3::Exp(delta.tail(state.dim_P() - state.dim_theta()));
            state.set_theta(state.get_theta() + delta.head(state.dim_theta()));
        } else {
            dX = SEk3::Exp(delta);
        }
        X = is_left? X * dX : dX * X;
        state.set_X(X);
        state.set_P(state.covariance_processing(P_new, frame_type, false));
    }
    if (b[3] != 0 && need_structure_update) {
        state.update_structure(Registry_->blocks);
    }
}
} // namespace inekf