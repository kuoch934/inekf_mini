#ifndef CORRECTION_H
#define CORRECTION_H

#include "inekf_mini/types.h"
#include <type_traits>

namespace inekf {
class CorrectionRegistry;
// --- 修正塊基底類別 ---
class CorrectionBlock { // for non-dType observation, which does not change the structure of state and covariance
public:
    virtual ~CorrectionBlock() = default;
    FrameType frame_type;
    Eigen::Vector3d b = Eigen::Vector3d::Zero(); // correction model [R v p] => [bR bv bp]
    struct Observation {
        Eigen::Vector3d outer_ref = Eigen::Vector3d::Zero();
        const char* name = nullptr;
        Eigen::Vector3d data = Eigen::Vector3d::Zero();
        Eigen::Matrix3d noise = Eigen::Matrix3d::Zero();

        // 支援obs.resize(n);預先置觀測數量，及動態生成
        Observation() = default;
        Observation(const char* n) : name(n) {}
    };
    std::vector<Observation> obs;

    void correct(RobotState& state);
};

class DynamicCorrectionBlock { // for dType observation, which may change the structure of state and covariance
protected:
    CorrectionRegistry* Registry_ = nullptr;
public:
    virtual ~DynamicCorrectionBlock() = default;
    void setRegistry(CorrectionRegistry* mgr) { Registry_ = mgr; }
    FrameType frame_type;
    Eigen::Vector4d b = Eigen::Vector4d::Zero(); // correction model [R v p d] => [bR bv bp bd]
    Eigen::Matrix3d process_noise = Eigen::Matrix3d::Zero(); // process noise for the new d state
    struct Observation {
        Eigen::Vector3d outer_ref = Eigen::Vector3d::Zero();
        const char* name = nullptr;
        Eigen::Vector3d data = Eigen::Vector3d::Zero();
        Eigen::Matrix3d noise = Eigen::Matrix3d::Zero();
        bool is_active = false;
        std::shared_ptr<int> index_ptr = std::make_shared<int>(-1);
        
        Observation() = default;
        Observation(const char* n) : name(n) {}
    };
    std::vector<Observation> obs;

    void correct(RobotState& state);
};

class CorrectionRegistry {
public: // CorrectionBlock 純生成，DynamicCorrectionBlock 會掛載到 blocks(Registry) 後被 RobotState 查閱並調整矩陣結構
    std::vector<std::shared_ptr<DynamicCorrectionBlock>> blocks;
    template <typename T, typename... Args>
    std::shared_ptr<T> addBlock(Args&&... args) {
        auto new_block = std::make_shared<T>(std::forward<Args>(args)...);
        if constexpr (std::is_base_of_v<DynamicCorrectionBlock, std::decay_t<T>>) {
            new_block->setRegistry(this);
            blocks.push_back(new_block);
        } 
        return new_block;
    }
};
} // namespace inekf

#endif // CORRECTION_H