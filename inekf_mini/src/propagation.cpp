
#include "inekf_mini/robot_state.h"
#include "inekf_mini/math/lie_group.hpp"

namespace {
  struct PropagationTerms {
      Eigen::Matrix3d G0, G1, G2, G3; // Gamma matrices 
      Eigen::Matrix3d G0t, G1t, G2t, G3t; // Gamma matrices transposes
      Eigen::Matrix3d Psi1, Psi2; // bias coupling terms (Psi)
  };

  const double TOLERANCE = 1e-6;

  long factorial(int n) {
      static const long table[] = {1, 1, 2, 6, 24, 120, 720, 5040, 40320};
      return (n >= 0 && n <= 8) ? table[n] : 1; 
  }
}

Eigen::Matrix3d inekf::GammaSO3(const Eigen::Vector3d& w, int m) {
  assert(m >= 0);
  const double theta = w.norm();
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d A = inekf::SO3::skew(w);
  const Eigen::Matrix3d A2 = A * A;

  if (theta < TOLERANCE) {
      return (1.0 / factorial(m)) * I + (1.0 / factorial(m + 1)) * A + (1.0 / factorial(m + 2)) * A2;
  }

  const double theta2 = theta * theta;
  const double theta3 = theta2 * theta;

  switch (m) {
  case 0:
      return I + (std::sin(theta) / theta) * A + ((1.0 - std::cos(theta)) / theta2) * A2;

  case 1:
      return I + ((1.0 - std::cos(theta)) / theta2) * A + ((theta - std::sin(theta)) / theta3) * A2;

  case 2:
      return 0.5 * I + ((theta - std::sin(theta)) / theta3) * A 
              + (theta2 + 2.0 * std::cos(theta) - 2.0) / (2.0 * theta2 * theta2) * A2;

  default:
      Eigen::Matrix3d R = I + (std::sin(theta) / theta) * A + ((1.0 - std::cos(theta)) / theta2) * A2;
      Eigen::Matrix3d S = I;
      Eigen::Matrix3d Ak = I;
      long kfact = 1;
      for (int k = 1; k <= m; ++k) {
          kfact *= k;
          Ak *= A;
          S += (1.0 / kfact) * Ak;
      }
      
      double inv_theta_m = 1.0 / std::pow(theta, m);
      int sign = ((m / 2) % 2 == 0) ? 1 : -1;
      if (m % 2 != 0) { // odd
          sign = (((m + 1) / 2) % 2 == 0) ? 1 : -1;
          return (1.0 / kfact) * I + (sign * inv_theta_m / theta) * A * (R - S);
      } else { // even
          return (1.0 / kfact) * I + (sign * inv_theta_m) * (R - S);
      }
  }
}

namespace {
  using namespace inekf;
  void compute_the_bias_terms(const Eigen::Vector3d& w, const Eigen::Vector3d& a, double dt, PropagationTerms& G) {
      // Compute the complicated bias terms (Psi)
      Eigen::Matrix3d ax = SO3::skew(a);
      Eigen::Matrix3d wx = SO3::skew(w);
      Eigen::Matrix3d wx2 = wx * wx;
      double dt2 = dt * dt;
      double dt3 = dt2 * dt;
      double theta = w.norm();

      /// TODO: Get better approximation using taylor series when theta < tol
      if (theta < TOLERANCE) {
          G.Psi1 = 0.5 * ax * dt2;
          G.Psi2 = (1.0 / 6.0) * ax * dt3;
          return;
      }
      
      double theta2 = theta * theta;
      double theta3 = theta2 * theta;
      double theta4 = theta3 * theta;
      double theta5 = theta4 * theta;
      double theta6 = theta5 * theta;
      double theta7 = theta6 * theta;
      double thetadt = theta * dt;
      double thetadt2 = thetadt * thetadt;
      double thetadt3 = thetadt2 * thetadt;
      double sinthetadt = std::sin(thetadt);
      double costhetadt = std::cos(thetadt);
      double sin2thetadt = std::sin(2 * thetadt);
      double cos2thetadt = std::cos(2 * thetadt);
      double thetadtcosthetadt = thetadt * costhetadt;
      double thetadtsinthetadt = thetadt * sinthetadt;

      G.Psi1
          = ax * G.G2t * dt2
          + ((sinthetadt - thetadtcosthetadt) / (theta3)) * (wx * ax)
          - ((cos2thetadt - 4 * costhetadt + 3) / (4 * theta4)) * (wx * ax * wx)
          + ((4 * sinthetadt + sin2thetadt - 4 * thetadtcosthetadt - 2 * thetadt) / (4 * theta5)) * (wx * ax * wx2)
          + ((thetadt2 - 2 * thetadtsinthetadt - 2 * costhetadt + 2) / (2 * theta4)) * (wx2 * ax)
          - ((6 * thetadt - 8 * sinthetadt + sin2thetadt) / (4 * theta5)) * (wx2 * ax * wx)
          + ((2 * thetadt2 - 4 * thetadtsinthetadt - cos2thetadt + 1) / (4 * theta6)) * (wx2 * ax * wx2);

      G.Psi2
          = ax * G.G3t * dt3
          - ((thetadtsinthetadt + 2 * costhetadt - 2) / (theta4)) * (wx * ax)
          - ((6 * thetadt - 8 * sinthetadt + sin2thetadt) / (8 * theta5)) * (wx * ax * wx)
          - ((2 * thetadt2 + 8 * thetadtsinthetadt + 16 * costhetadt + cos2thetadt - 17) / (8 * theta6)) * (wx * ax * wx2)
          + ((thetadt3 + 6 * thetadt - 12 * sinthetadt + 6 * thetadtcosthetadt) / (6 * theta5)) * (wx2 * ax)
          - ((6 * thetadt2 + 16 * costhetadt - cos2thetadt - 15) / (8 * theta6)) * (wx2 * ax * wx)
          + ((4 * thetadt3 + 6 * thetadt - 24 * sinthetadt - 3 * sin2thetadt + 24 * thetadtcosthetadt) / (24 * theta7)) * (wx2 * ax * wx2);
  }

  void compute_propagation_terms(const Eigen::Vector3d& w, const Eigen::Vector3d& a, const double dt, PropagationTerms& G){
      Eigen::Vector3d phi = w * dt;
      G.G0 = GammaSO3(phi, 0);
      G.G1 = GammaSO3(phi, 1);
      G.G2 = GammaSO3(phi, 2);
      G.G0t = G.G0.transpose();
      G.G1t = G.G1.transpose();
      G.G2t = G.G2.transpose();
      G.G3t = GammaSO3(-phi, 3);
      compute_the_bias_terms(w, a, dt, G);
  }
} // namespace

void inekf::RobotState::propagate(const Eigen::Vector3d& w_raw, const Eigen::Vector3d& a_raw, double dt) {
  // 座標系轉換：將 IMU 數據轉到 Base Frame
  Eigen::Vector3d w = R_imu_in_base * (w_raw - (bias_estimate_enabled() ? get_bg() : Eigen::Vector3d::Zero()));
  Eigen::Vector3d a = R_imu_in_base * (a_raw - (bias_estimate_enabled() ? get_ba() : Eigen::Vector3d::Zero()))
   - w.cross(w.cross(p_imu_in_base));

  Eigen::Matrix3d R = get_rotation();
  Eigen::Vector3d v = get_velocity();
  Eigen::Vector3d p = get_position();

  Eigen::MatrixXd P = get_P();
  Eigen::MatrixXd Q_bar = get_Q();
  
  PropagationTerms G;
  compute_propagation_terms(w, a, dt, G);
  double dt2 = dt * dt;

  // state propagation
  if (frame_type == FrameType::World) { 
    // World frame
    X_.block<3, 3>(0, 0) = R*G.G0;
    X_.block<3, 1>(0, 3) = v + (R*G.G1*a + g)*dt;
    X_.block<3, 1>(0, 4) = p + v*dt + (R*G.G2*a + 0.5*g)*dt2;
  } else { 
    // Body frame
    X_.block<3, 3>(0, 0) = G.G0t*R;
    X_.block<3, 1>(0, 3) = G.G0t*(v - R*g*dt - G.G1*a*dt);
    X_.block<3, 1>(0, 4) = G.G0t*(p - v*dt + (0.5*R*g - G.G2*a)*dt2);
    for (int i = BASE_DIM_X; i < dim_X(); ++i) {
      X_.block<3, 1>(0, i) = G.G0t * get_vector(i); // Propagate other states (e.g., landmarks)
    }
  }

  bool A_is_right = false;
  // process noise preprocessing
  bool cond1 = (frame_type == FrameType::World && error_type == ErrorType::RightInvariant);
  bool cond2 = (frame_type == FrameType::Body  && error_type == ErrorType::LeftInvariant);
  if (cond1 || cond2) {
    A_is_right = true;
    Eigen::MatrixXd Adj_X = cond1 ? SEk3::adjoint(X_) : SEk3::adjoint_inv(X_);
    int dim_xi = dim_P() - DIM_THETA;
    Q_bar.block(DIM_THETA, DIM_THETA, dim_xi, dim_xi) = Adj_X * Q_bar.block(DIM_THETA, DIM_THETA, dim_xi, dim_xi) * Adj_X.transpose();
  }

  // state transition matrix
  Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(dim_P(), dim_P());
  if (A_is_right) {
    // Gamma_A0
    Eigen::Matrix3d gx = SO3::skew(g);
    Phi.block<3, 3>(DIM_THETA + 3, DIM_THETA) = gx * dt;
    Phi.block<3, 3>(DIM_THETA + 6, DIM_THETA) = 0.5 * gx * dt2;
    Phi.block<3, 3>(DIM_THETA + 6, DIM_THETA + 3) = Eigen::Matrix3d::Identity() * dt;
    // Gamma_A1
    if (bias_estimate_enabled()) {
      Eigen::Matrix3d RG1dt = R * G.G1 * dt;
      Phi.block<3, 3>(DIM_THETA, 0) = -RG1dt;                   // Phi_15
      Phi.block<3, 3>(DIM_THETA + 3, 0) = -SO3::skew(X_.block<3, 1>(0, 3)) * RG1dt + R * G.Psi1;    // Phi_25
      Phi.block<3, 3>(DIM_THETA + 6, 0) = -SO3::skew(X_.block<3, 1>(0, 4)) * RG1dt + R * G.Psi2;    // Phi_35
      for (int i = BASE_DIM_X; i < dim_X(); ++i) { 
        Phi.block<3, 3>(P_Xvecidx(i), 0) = -SO3::skew(X_.block<3, 1>(0, i)) * RG1dt;   // Phi_(3+i)5
      }
      Phi.block<3, 3>(DIM_THETA + 3, 3) = -RG1dt;               // Phi_26
      Phi.block<3, 3>(DIM_THETA + 6, 3) = -R*G.G2*dt2;          // Phi_36
    }
  } else {
    // Gamma_A0
    Phi.block<3, 3>(DIM_THETA, DIM_THETA) = G.G0t;              // Phi_11
    Phi.block<3, 3>(DIM_THETA + 3, DIM_THETA + 3) = G.G0t;      // Phi_22
    Phi.block<3, 3>(DIM_THETA + 6, DIM_THETA + 6) = G.G0t;      // Phi_33
    for (int i = BASE_DIM_X; i < dim_X(); ++i) { 
      Phi.block<3, 3>(P_Xvecidx(i), P_Xvecidx(i)) = G.G0t;      // Phi_(3+i)(3+i)
    }   
    Phi.block<3, 3>(DIM_THETA + 3, DIM_THETA) = -G.G0t * SO3::skew(G.G1 * a) * dt;     // Phi_21
    Phi.block<3, 3>(DIM_THETA + 6, DIM_THETA) = -G.G0t * SO3::skew(G.G2 * a) * dt2;    // Phi_31
    Phi.block<3, 3>(DIM_THETA + 6, DIM_THETA + 3) = G.G0t * dt;                        // Phi_32
    // Gamma_A1
    if (bias_estimate_enabled()) {
      Phi.block<3, 3>(DIM_THETA, 0) = -G.G1t * dt;              // Phi_15
      Phi.block<3, 3>(DIM_THETA + 3, 0) = G.G0t * G.Psi1;       // Phi_25
      Phi.block<3, 3>(DIM_THETA + 6, 0) = G.G0t * G.Psi2;       // Phi_35
      Phi.block<3, 3>(DIM_THETA + 3, 3) = -G.G1t * dt;          // Phi_26
      Phi.block<3, 3>(DIM_THETA + 6, 3) = -G.G0t * G.G2 * dt2;  // Phi_36
    }
  } 
  // discrete-time covariance update
  P = Phi * P * Phi.transpose() + Q_bar * dt;
  P = 0.5 * (P + P.transpose()).eval(); // Ensure symmetry
  set_P(P);
}