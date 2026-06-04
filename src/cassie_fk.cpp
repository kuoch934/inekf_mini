// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <nav_msgs/msg/odometry.hpp>

// // 假設這是你自己重構的 InEKF 核心類別

// #include "inekf_mini/filter.h"

// class Contact : public inekf::DynamicCorrectionBlock {
// public:

// };

// class inekfNode : public rclcpp::Node {
// public:
//     inekfNode() : Node("inekf_node") {
//         // 1. 初始化 Subscriber (接收高頻 IMU)
//         imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
//             "/imu/data", 10, std::bind(&inekfNode::imu_callback, this, std::placeholders::_1));

//         // 2. 初始化 Publisher (發布估測位姿)
//         odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/aria/odom", 10);

//         // 3. 建立定時器 (主執行緒：負責初始化檢查與 RunOnce)
//         // 設定 1kHz (1ms) 的執行頻率
//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(1), std::bind(&inekfNode::loop, this));

//         RCLCPP_INFO(this->get_logger(), "Estimator Node Started. Waiting for data...");
//     }

// private:
//     // --- ROS 2 通訊組件 ---
//     rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
//     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
//     rclcpp::TimerBase::SharedPtr timer_;

//     // --- 核心估測組件 ---
//     // inEKF filter_; // 你自己寫的 InEKF 類別
//     inekf::RobotState robot_state{load_config("config/config.yaml"), inekf::FrameType::World, inekf::ErrorType::RightInvariant};

//     // --- 狀態變數 ---
//     bool calibrated_ = false;
//     bool welford_flg = true; // 用於控制 Welford 重置
//     Eigen::Matrix3d R_imu2body_ = Eigen::Matrix3d::Identity(); // 假設 IMU 已經對齊，否則需要根據實際安裝調整
//     // const int INIT_WINDOW = 500; // 需要 500 筆數據來初始化
//     inekf::Welford<3> gyro_welford_;
//     inekf::Welford<3> acc_welford_;

//     inekf::CorrectionRegistry registry_; // 用於管理不同類型的修正塊

//     void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
//         Eigen::Vector3d raw_acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
//         Eigen::Vector3d raw_gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

//         Eigen::Vector3d acc_b = R_imu2body_ * raw_acc;
//         Eigen::Vector3d gyro_b = R_imu2body_ * raw_gyro;
        
//         if (welford_flg){
//             gyro_welford_.reset();
//             acc_welford_.reset();
//             welford_flg = false;
//         }
//         gyro_welford_.update(gyro_b);
//         acc_welford_.update(acc_b);

//         if (!calibrated_) {
//             if (acc_welford_.count() >= 500) {
//                 robot_state.reset(gyro_welford_.mean(), acc_welford_.mean(), gyro_welford_.variance(), acc_welford_.variance());
//                 calibrated_ = true;
//             }
//         } else {
//             gyro_welford_.n_ = 499; // 讓 Welford 不再更新，保持在校準狀態
//             acc_welford_.n_ = 499; 
//         }
    

//     // --- 主邏輯迴圈 (Thread B: 邏輯管理) ---
//     void loop() {
//         if (!calibrated_) {

//             // // Calibration Step
//             // std::lock_guard<std::mutex> lock(buffer_mutex_);
//             // if (imu_buffer_.size() < INIT_WINDOW) return;

//             // if (filter_.calibrate(imu_buffer_)) {
//             //     calibrated_ = true;
//             //     imu_buffer_.clear();
//             //     RCLCPP_INFO(this->get_logger(), "Calibrate Successfully!");
//             // }
//             // return;

//         } else {

//             std::deque<sensor_msgs::msg::Imu::SharedPtr> local_queue;

//             // // 快速取出所有數據，減少對 Callback 執行緒的阻塞
//             // {
//             //     std::lock_guard<std::mutex> lock(buffer_mutex_);
//             //     if (imu_buffer_.empty()) return;
//             //     local_queue.swap(imu_buffer_); 
//             // }

//             // 處理這段時間內累積的所有 IMU 數據
//             rclcpp::Time last_stamp;
//             while (!local_queue.empty()) {
//                 auto imu_msg = local_queue.front();
//                 last_stamp = imu_msg->header.stamp;

//                 // 執行 InEKF 傳播步 (Propagate)
//                 robot_state.propagate(gyro_b, acc_b, dt);

//                 local_queue.pop_front();
//             }

//             // 批次處理完後，發布最新的狀態
//             publish_odometry(last_stamp);

//         }
//     }

    

//     void publish_odometry(const rclcpp::Time& stamp) {
//         nav_msgs::msg::Odometry odom;
//         odom.header.stamp = stamp;
//         odom.header.frame_id = "odom";
        
//         // 從 robot_state 獲取李群狀態矩陣 X 並轉為 ROS 格式
//         auto X = robot_state.get_X(); 
//         odom.pose.pose.position.x = X(0, 4); // 假設你的矩陣排列
//         // ... 其他填充邏輯
        
//         odom_pub_->publish(odom);
//     }
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<inekfNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }