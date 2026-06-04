#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

using namespace inekf;

class CassieKinematicsCorrection : public DynamicCorrectionBlock {
public:
    CassieKinematicsCorrection(const std::string& urdf_path) {

        b << 0.0, 0.0, 1.0, -1.0; 
        frame_type = FrameType::Body; // 觀測量 data 是在 Body Frame (IMU) 下定義的
        process_noise = Eigen::Matrix3d::Identity() * 1e-5; // 足端狀態 d 的隨機遊走噪聲

        // 1. 初始化 Pinocchio 模型 (FreeFlyer)
        pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), model_);
        data_ = pinocchio::Data(model_);

        // 2. 獲取 Frame ID
        pelvis_id_ = model_.getFrameId("pelvis");
        left_toe_id_  = model_.getFrameId("left_toe");
        right_toe_id_ = model_.getFrameId("right_toe");

        // 3. 預設關節映射 (14 個廣義座標索引)
        std::vector<std::string> joint_names = {
            "hip_abduction_left", "hip_rotation_left", "hip_flexion_left", "knee_joint_left", 
            "knee_to_shin_left", "ankle_joint_left", "toe_joint_left",
            "hip_abduction_right", "hip_rotation_right", "hip_flexion_right", "knee_joint_right", 
            "knee_to_shin_right", "ankle_joint_right", "toe_joint_right"
        };
        for (const auto& name : joint_names) {
            q_indices_.push_back(model_.idx_qs[model_.getJointId(name)]);
        }

        // 4. 初始化 Observation (左腳 index 0, 右腳 index 1)
        obs.resize(2);
        obs[0].name = "left_foot";
        obs[1].name = "right_foot";
    }

    // 此處接收 raw encoder 並更新內部觀測值
    void set_measurements(const Eigen::VectorXd& q_raw, const Eigen::Vector2i& contact) {
        // 映射 q (35維)
        Eigen::VectorXd q = pinocchio::neutral(model_);
        for (size_t i = 0; i < 14; ++i) {
            int idx = q_indices_[i];
            q[idx]     = std::cos(q_raw[i]);
            q[idx + 1] = std::sin(q_raw[i]);
        }

        // 2. 進行運動學計算與雅可比計算
        pinocchio::computeJointJacobians(model_, data_, q); // 計算全域 Jacobian
        pinocchio::updateFramePlacements(model_, data_);
        
        const auto& oM_pelvis = data_.oMf[pelvis_id_];

        for (int i = 0; i < 2; ++i) {
        pinocchio::FrameIndex toe_id = (i == 0) ? left_toe_id_ : right_toe_id_;
        const auto& oM_toe = data_.oMf[toe_id];

        // 計算 Body Frame 下的相對位移： p_rel = R_p.T * (p_toe - p_pelvis)
        // 也就是 Pelvis 到 Toe 的向量在 Pelvis Frame 下的表示
        obs[i].data = oM_pelvis.actInv(oM_toe).translation(); 

        // 3. 計算 Jacobian (切換到 LOCAL 參考系以匹配 Body Frame 觀測)
        Eigen::Matrix<double, 6, Eigen::Dynamic> J_local(6, model_.nv);
        pinocchio::getFrameJacobian(model_, data_, toe_id, pinocchio::ReferenceFrame::LOCAL, J_local);

        // 取得平移部分相對於關節速度的 Jacobian (忽略 Base 速度的前 6 列)
        Eigen::Matrix3d R_oP = oM_pelvis.rotation().transpose();
        Eigen::Matrix<double, 3, 14> J_toe_body = R_oP *J_local.block(0, 6, 3, 14);

        // 4. 協方差傳播
        double q_var = 1e-3; 
        obs[i].noise = J_toe_body * (Eigen::VectorXd::Constant(14, q_var).asDiagonal()) * J_toe_body.transpose();
        obs[i].noise += Eigen::Matrix3d::Identity() * 1e-6; 
        obs[i].is_active = (contact[i] > 0);
    }
    }

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    std::vector<int> q_indices_;
    pinocchio::FrameIndex pelvis_id_, left_toe_id_, right_toe_id_;
};