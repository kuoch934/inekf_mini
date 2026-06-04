#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>

#include "inekf_mini/robot_state.h"
#include "inekf_mini/correction.h"
#include "inEKF_mini/cassie_kinematics_block.h" // 包含你定義的 CassieKinematicsCorrection

// colcon build --packages-select inEKF_mini --symlink-install
// source install/setup.bash
// ros2 run inEKF_mini cassie_inekf_app

// colcon build --packages-select inEKF_mini --cmake-args -DCMAKE_BUILD_TYPE=Release
// 如果你只是用一般的 colcon build，它預設是 Debug 模式，這會導致 Eigen 矩陣運算慢了 10 倍。請嘗試用 Release 模式重新編譯：

using namespace inekf;

struct InEKFData {
    double t;
    Eigen::Vector3d gyro, acc;
    Eigen::VectorXd enc;
    Eigen::Vector2i contact;
};

// 輔助函式：一次讀完 CSV 減少 I/O 中斷
std::vector<InEKFData> preloadData(const std::string& root) {
    std::vector<InEKFData> data_stream;
    std::ifstream f_t(root + "data/time.csv"), f_g(root + "data/imu_gyro.csv"), f_a(root + "data/imu_acc.csv");
    std::ifstream f_e(root + "data/encoders.csv"), f_c(root + "data/contact.csv");
    
    auto getLineVec = [](std::ifstream& f) -> std::vector<double> {
        std::string line;
        if (!std::getline(f, line)) return {};
        std::stringstream ss(line);
        std::vector<double> v;
        std::string s;
        while (std::getline(ss, s, ',')) v.push_back(std::stod(s));
        return v;
    };

    while (f_t.peek() != EOF) {
        auto tv = getLineVec(f_t); if (tv.empty()) break;
        auto gv = getLineVec(f_g);
        auto av = getLineVec(f_a);
        auto ev = getLineVec(f_e);
        auto cv = getLineVec(f_c);

        InEKFData d;
        d.t = tv[0];
        d.gyro = Eigen::Vector3d(gv[0], gv[1], gv[2]);
        d.acc = Eigen::Vector3d(av[0], av[1], av[2]);
        d.enc = Eigen::Map<Eigen::VectorXd>(ev.data(), ev.size());
        d.contact = Eigen::Vector2i((int)cv[0], (int)cv[1]);
        data_stream.push_back(d);
    }
    return data_stream;
}

int main() {
    std::string root_path = "src/inEKF_mini/";
    std::cout << "[INFO] 正在預載入數據..." << std::endl;
    auto data_stream = preloadData(root_path);
    if (data_stream.empty()) { std::cerr << "載入失敗！" << std::endl; return 1; }

    // 2. 初始化
    RobotStateConfig config;
    config.bias_estimate_enable = true;
    config.R_imu_in_base << 1, 0, 0, 0, -1, 0, 0, 0, -1; 
    // config.p_imu_in_base << 0.03155, 0, -0.07996;
    config.p_imu_in_base << 0, 0, 0;
    auto state = std::make_shared<RobotState>(config, FrameType::World, ErrorType::RightInvariant);
    
    CorrectionRegistry registry;
    std::string urdf = root_path + "cassie_description/urdf/cassie.urdf";
    auto cassie_leg = registry.addBlock<CassieKinematicsCorrection>(urdf);

    state->set_position(Eigen::Vector3d(0, 0, 1));

    // // 3. 準備結果緩衝區 (避免迴圈內寫入檔案)
    // std::stringstream res_ss;
    // res_ss << "p_x,p_y,p_z,v_x,v_y,v_z,r11,r12,r13,r21,r22,r23,r31,r32,r33\n";

    double last_t = data_stream[0].t;
    std::cout << "[START] 狀態估測開始運行..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();

    // 4. 主循環 (純計算模式)
    for (size_t step = 0; step < data_stream.size(); ++step) {
        const auto& data = data_stream[step];
        double dt = (step == 0) ? 0.0005 : (data.t - last_t);
        last_t = data.t;

        // A. 傳播與修正
        state->propagate(data.gyro, data.acc, dt);

        cassie_leg->set_measurements(data.enc, data.contact);
        cassie_leg->correct(*state);

        // // B. 緩衝結果
        // if (step % 1 == 0) { // 可以調整儲存頻率
        //     const auto& p = state->get_position();
        //     const auto& v = state->get_velocity();
        //     const auto& R = state->get_rotation();
        //     res_ss << p.x() << "," << p.y() << "," << p.z() << ","
        //            << v.x() << "," << v.y() << "," << v.z() << ","
        //            << R(0,0) << "," << R(0,1) << "," << R(0,2) << ","
        //            << R(1,0) << "," << R(1,1) << "," << R(1,2) << ","
        //            << R(2,0) << "," << R(2,1) << "," << R(2,2) << "\n";
        // }

        // if (step % 1000 == 0) {
        //     std::cout << "\r處理中: " << (step * 100 / data_stream.size()) << "%" << std::flush;
        // }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;

    std::cout << "\n[DONE] 處理完成！" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "總處理時間: " << diff.count() << " 秒" << std::endl;
    std::cout << "數據總時長: " << data_stream.back().t - data_stream.front().t << " 秒" << std::endl;
    std::cout << "實時率 (RTF): " << diff.count() / (data_stream.back().t - data_stream.front().t) << " (越小越快)" << std::endl;
    std::cout << "========================================" << std::endl;

    // // 寫入 CSV
    // std::cout << "\n[INFO] 正在將結果寫入 CSV..." << std::endl;
    // std::ofstream res_file(root_path + "src/inekf_result.csv");
    // res_file << res_ss.str();
    // std::cout << "\n[INFO] 結果已寫入 CSV" << std::endl;
    // res_file.close();
    return 0;
}