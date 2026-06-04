#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "inekf_mini/robot_state.h"
#include "inekf_mini/correction.h"
#include "inekf_mini/math/welford.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

using namespace inekf;

// ==========================================
// 1. 定義人形機器人的運動學修正塊 (雙足)
// ==========================================
class CassieKinematicsCorrection : public DynamicCorrectionBlock {
public:
    CassieKinematicsCorrection(const std::string& urdf_path) {
        // 1. 初始化 Pinocchio 模型 (FreeFlyer)
        pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), model_);
        data_ = pinocchio::Data(model_);

        // 2. 獲取 Frame ID
        vectornav_id_ = model_.getFrameId("vectorNav");
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
        
        // 設定觀測雜訊 (Cassie 實務上約 0.01m ~ 0.05m)
        obs[0].noise = Eigen::Matrix3d::Identity() * 0.01;
        obs[1].noise = Eigen::Matrix3d::Identity() * 0.01;
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

        // Pinocchio FK
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);

        const auto& oM_v = data_.oMf[vectornav_id_];
        
        // 更新左腳觀測
        obs[0].data = oM_v.actInv(data_.oMf[left_toe_id_]).translation();
        obs[0].is_active = (contact[0] > 0);

        // 更新右腳觀測
        obs[1].data = oM_v.actInv(data_.oMf[right_toe_id_]).translation();
        obs[1].is_active = (contact[1] > 0);
    }

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    std::vector<int> q_indices_;
    pinocchio::FrameIndex vectornav_id_, left_toe_id_, right_toe_id_;
};

// ==========================================
// 2. 狀態估測 ROS 2 節點
// ==========================================
class InEKFNode : public rclcpp::Node {
public:
    InEKFNode() : Node("inekf_estimator_node") {
        // 1. 初始化設定與狀態
        RobotStateConfig config;
        config.bias_estimate_enable = true;
        config.rot_init_std = 0.01;
        config.vel_init_std = 0.1;
        config.pos_init_std = 0.01;
        
        // 採用 World Frame 與 Right-Invariant，適合人形機器人底盤估測
        state_ = std::make_shared<RobotState>(config, FrameType::World, ErrorType::RightInvariant);

        std::string urdf_path = "src/inEKF_mini/cassie_description/urdf/cassie.urdf";

        // 2. 註冊修正塊
        cassie_block_ = registry_.addBlock<CassieKinematicsCorrection>(urdf_path);

        // 3. ROS 通訊介面
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 1000, std::bind(&InEKFNode::imu_callback, this, std::placeholders::_1));
        
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 100, std::bind(&InEKFNode::joint_callback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/inekf/odometry", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(), "InEKF Humanoid Estimator Initialized.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    

    /* inekf */
    std::shared_ptr<RobotState> state_;
    CorrectionRegistry registry_;
    // propagation
    rclcpp::Time last_imu_time_;
    bool is_calibrated_ = false;
    inekf::Welford<3> w_welford_; // angular_velocity
    inekf::Welford<3> a_welford_; // linear_acceleration
    double imu_tau = 0.1; // 100ms 的特徵時間
    // correction blocks
    std::shared_ptr<CassieKinematicsCorrection> cassie_block_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        rclcpp::Time current_time = msg->header.stamp;
        Eigen::Vector3d w_raw(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        Eigen::Vector3d a_raw(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

        if (!is_calibrated_) {
            if (w_welford_.count() < 500) {
                w_welford_.update(w_raw);
                a_welford_.update(a_raw);
            } else {
                state_->reset(w_welford_.mean(), a_welford_.mean(), w_welford_.variance(), a_welford_.variance());
                last_imu_time_ = msg->header.stamp;
                is_calibrated_ = true;
                // print calibration results
                Eigen::Vector3d wb = w_welford_.mean();
                Eigen::Vector3d ab = a_welford_.mean();
                Eigen::Vector3d w_var = w_welford_.variance();
                Eigen::Vector3d a_var = a_welford_.variance();
                RCLCPP_INFO(this->get_logger(), "--- InEKF Calibration Complete ---");
                RCLCPP_INFO(this->get_logger(), "Gyro  Bias: [%.4f, %.4f, %.4f]", wb.x(), wb.y(), wb.z());
                RCLCPP_INFO(this->get_logger(), "Gyro  Var : [%.4f, %.4f, %.4f]", w_var.x(), w_var.y(), w_var.z());
                RCLCPP_INFO(this->get_logger(), "Accel Mean: [%.4f, %.4f, %.4f] (Gravity included)", ab.x(), ab.y(), ab.z());
                RCLCPP_INFO(this->get_logger(), "Accel Var : [%.4f, %.4f, %.4f]", a_var.x(), a_var.y(), a_var.z());
            }            
            return;
        }
        double dt = (current_time - last_imu_time_).seconds();
        last_imu_time_ = current_time;
        if (dt <= 0 || dt > 0.1) return;
        // dynamic noise estimation
        double adaptive_alpha = std::min(dt / imu_tau, 1.0);
        w_welford_.update_weighted(w_raw, adaptive_alpha);
        a_welford_.update_weighted(a_raw, adaptive_alpha);
        // Propagation
        state_->set_Q_imu(w_welford_.variance(), a_welford_.variance());
        state_->propagate(w_raw, a_raw, dt);
        publish_state(current_time);
    }

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!is_calibrated_) return;

        // --- 1. 準備數據 ---
        // 將 ROS msg 轉為 Eigen (需注意 Cassie URDF 14 個關節的對應順序)
        Eigen::VectorXd q_raw = Eigen::VectorXd::Zero(14);
        for(size_t i=0; i<14; ++i) {
            q_raw[i] = msg->position[i]; 
        }

        // --- 2. 處理接觸狀態 ---
        // 真實 Cassie 通常透過足端壓感或 joint_torque 判定
        // 這裡暫時維持簡單邏輯或從特定 topic 讀取
        Eigen::Vector2i contact(0, 0);
        // 假設你有判定邏輯，例如：if(force > threshold) contact[0] = 1;
        
        // --- 3. 執行運動學計算與 InEKF 修正 ---
        // 這會觸發你寫在 CassieKinematicsCorrection 裡的 Pinocchio FK
        cassie_block_->set_measurements(q_raw, contact);

        // 自動執行結構檢查與 Kalman 更新
        cassie_block_->correct(*state_);
    }

    void publish_state(const rclcpp::Time& stamp) {
        Eigen::Matrix3d R = state_->get_rotation();
        Eigen::Vector3d p = state_->get_position();
        Eigen::Vector3d v = state_->get_velocity();
        Eigen::Quaterniond q(R);

        // Publish Odometry
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = stamp;
        odom.header.frame_id = "world";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = p.x();
        odom.pose.pose.position.y = p.y();
        odom.pose.pose.position.z = p.z();
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();

        odom.twist.twist.linear.x = v.x();
        odom.twist.twist.linear.y = v.y();
        odom.twist.twist.linear.z = v.z();

        odom_pub_->publish(odom);

        // Publish TF
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = "world";
        t.child_frame_id = "base_link";
        t.transform.translation.x = p.x();
        t.transform.translation.y = p.y();
        t.transform.translation.z = p.z();
        t.transform.rotation.w = q.w();
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        
        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InEKFNode>();
    
    // 考慮人形機器人高頻 IMU (500Hz~1kHz) 的特性，建議使用 MultiThreadedExecutor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}