from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def rotation_to_euler_hartley(R):
    """
    將旋轉矩陣 R 轉換為尤拉角 (Roll, Pitch, Yaw)。
    慣例：Z(Yaw) @ Y(Pitch) @ X(Roll)
    """
    if R[2,0] < 1:
        if R[2,0] > -1:
            qx = np.arctan2(R[2,1], R[2,2])
            qy = np.arcsin(-R[2,0])
            qz = np.arctan2(R[1,0], R[0,0])
        else:
            qx, qy, qz = 0.0, np.pi/2.0, -np.arctan2(-R[1,2], R[1,1])
    else:
        qx, qy, qz = 0.0, -np.pi/2.0, np.arctan2(-R[1,2], R[1,1])
    return np.array([qx, qy, qz])

# =================================================================================
# 1. 載入資料 (請確保路徑正確)
# =================================================================================
root_dir = Path(__file__).parent.parent
data_dir = root_dir / 'data'

# 載入 C++ 產出的結果


print(f"正在從 {data_dir} 載入 CSV 數據...")

# --- 2. 載入數據 ---
# 讀取 C++ InEKF 結果
try:
    cpp_res = pd.read_csv(Path(__file__).parent / 'inekf_result.csv')
    res_len = len(cpp_res)
except FileNotFoundError:
    print("錯誤：找不到 inekf_result.csv，請先執行 C++ 程序。")
    exit()

# 讀取真值與時間 (假設這些 CSV 檔案也放在 data/ 下)
pos_gt = pd.read_csv(data_dir / 'gt_pos.csv', header=None).values[:res_len]
vel_gt = pd.read_csv(data_dir / 'gt_vel.csv', header=None).values[:res_len]
att_gt_raw = pd.read_csv(data_dir / 'gt_att.csv', header=None).values[:res_len]
time_range = pd.read_csv(data_dir / 'time.csv', header=None).values.flatten()[:res_len]

# --- 3. 數據重塑與處理 ---
# 提取估計值
res_p = cpp_res[['p_x', 'p_y', 'p_z']].values
res_v = cpp_res[['v_x', 'v_y', 'v_z']].values
res_R = cpp_res[['r11','r12','r13','r21','r22','r23','r31','r32','r33']].values.reshape(-1, 3, 3)

# 重塑真值旋轉矩陣 (N, 9) -> (N, 3, 3)
att_gt = att_gt_raw.reshape(-1, 3, 3)

# 座標校正矩陣 (對齊 IMU 與世界座標系)

# 計算估計與真值的尤拉角 (RPY)
res_rpy = np.array([rotation_to_euler_hartley(R) for R in res_R]) * 180 / np.pi
gt_rpy = np.array([rotation_to_euler_hartley(R) for R in att_gt[:res_len]]) * 180 / np.pi

# =================================================================================
# 3. 繪圖：位置、速度、姿態
# =================================================================================
fig, axes = plt.subplots(3, 3, figsize=(18, 12))

# --- 第 1 欄：位置對比 ---
labels_p = ['X (m)', 'Y (m)', 'Z (m)']
for i in range(3):
    axes[i, 0].plot(time_range, pos_gt[:res_len, i], 'k--', label='GT', alpha=0.6)
    axes[i, 0].plot(time_range, res_p[:, i], 'r', label='C++ InEKF')
    axes[i, 0].set_ylabel(labels_p[i])
    axes[i, 0].legend()
    axes[i, 0].grid(True, alpha=0.3)
axes[0, 0].set_title("Position")

# --- 第 2 欄：速度對比 ---
labels_v = ['Vx (m/s)', 'Vy (m/s)', 'Vz (m/s)']
for i in range(3):
    axes[i, 1].plot(time_range, vel_gt[:res_len, i], 'k--', label='GT', alpha=0.6)
    axes[i, 1].plot(time_range, res_v[:, i], 'g', label='C++ InEKF')
    axes[i, 1].set_ylabel(labels_v[i])
    axes[i, 1].legend()
    axes[i, 1].grid(True, alpha=0.3)
axes[0, 1].set_title("Velocity")

# --- 第 3 欄：姿態對比 ---
labels_a = ['Roll (deg)', 'Pitch (deg)', 'Yaw (deg)']
for i in range(3):
    axes[i, 2].plot(time_range, gt_rpy[:, i], 'k--', label='GT', alpha=0.6)
    axes[i, 2].plot(time_range, res_rpy[:, i], 'b', label='C++ InEKF')
    axes[i, 2].set_ylabel(labels_a[i])
    axes[i, 2].legend()
    axes[i, 2].grid(True, alpha=0.3)
axes[0, 2].set_title("Attitude (RPY)")

plt.suptitle(f"InEKF C++ Performance Analysis (Steps: {res_len})")
plt.tight_layout()
plt.show()