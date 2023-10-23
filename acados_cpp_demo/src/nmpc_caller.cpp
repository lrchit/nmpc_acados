
#include "nmpc_caller.h"

// 下面这两个函数用来将Eigen类型与数组类型互相转化
void nmpc_caller::copy_Array_to_Eigen(VectorXd &target, double *source, int len,
                                      int startIndex_eigen,
                                      int startIndex_array) {
  for (int i = 0; i < len; i++) {
    target(i + startIndex_eigen) = source[i + startIndex_array];
  }
}

void nmpc_caller::copy_Eigen_to_Array(double *target, VectorXd &source, int len,
                                      int startIndex_array,
                                      int startIndex_eigen) {
  for (int i = 0; i < len; i++) {
    target[i + startIndex_array] = source(i + startIndex_eigen);
  }
}

// 构造函数对求解器进行初始化，基本没啥可改的
nmpc_caller::nmpc_caller() {
  // create a capsule according to the pre-defined model
  acados_ocp_capsule = srbd_acados_create_capsule();

  // optimizer
  status = srbd_acados_create(acados_ocp_capsule);

  // 判断一下是否初始化成功，失败则立即退出
  if (status) {
    printf("srbd_acados_create() optimater returned status %d. Exiting.\n",
           status);
    exit(1);
  }

  // some important structure of ocp
  nlp_config = srbd_acados_get_nlp_config(acados_ocp_capsule);
  nlp_dims = srbd_acados_get_nlp_dims(acados_ocp_capsule);
  nlp_in = srbd_acados_get_nlp_in(acados_ocp_capsule);
  nlp_out = srbd_acados_get_nlp_out(acados_ocp_capsule);

  horizons = nlp_dims->N;
  nx = *nlp_dims->nx;
  nu = *nlp_dims->nu;

  last_optimal_state.setZero(12 * horizons);
  last_optimal_control.setZero(12 * horizons);
}

nmpc_caller::~nmpc_caller() {}

// 为求解器传入当前状态和期望状态以及参数
void nmpc_caller::nmpc_setup(VectorXd _current_state, VectorXd _p_feet,
                             VectorXd _gait_data, VectorXd _desired_state) {

  double current_state[12], p_feet[12 * horizons], gait_data[4 * horizons],
      desired_state[12 * (horizons + 1)];

  copy_Eigen_to_Array(current_state, _current_state, 12, 0, 0);
  copy_Eigen_to_Array(p_feet, _p_feet, 12 * horizons, 0, 0);
  copy_Eigen_to_Array(gait_data, _gait_data, 4 * horizons, 0, 0);
  copy_Eigen_to_Array(desired_state, _current_state, 12, 0, 0);
  copy_Eigen_to_Array(desired_state, _desired_state, 12 * horizons, 12, 0);

  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx",
                                current_state);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx",
                                current_state);

  double lh[20], uh[20];
  for (int i = 0; i < horizons; i++) {
    for (int j = 0; j < 4; j++) {
      if (gait_data[i * 4 + j] == 0) {
        lh[5 * j + 0] = 0;
        lh[5 * j + 1] = 0;
        lh[5 * j + 2] = 0;
        lh[5 * j + 3] = 0;
        lh[5 * j + 4] = 0;

        uh[5 * j + 0] = 0;
        uh[5 * j + 1] = 0;
        uh[5 * j + 2] = 0;
        uh[5 * j + 3] = 0;
        uh[5 * j + 4] = 0;
      } else {
        lh[5 * j + 0] = -10000;
        lh[5 * j + 1] = -10000;
        lh[5 * j + 2] = 0;
        lh[5 * j + 3] = 0;
        lh[5 * j + 4] = 0;

        uh[5 * j + 0] = 0;
        uh[5 * j + 1] = 0;
        uh[5 * j + 2] = 10000;
        uh[5 * j + 3] = 10000;
        uh[5 * j + 4] = 200;
      }
    }
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lh", lh);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "uh", uh);
  }

  for (int j = 0; j < horizons; j++) {

    double state_and_control_ref[24];
    double p_feet_and_gait_state[17];
    for (int k = 0; k < 12; k++) {
      state_and_control_ref[k] = desired_state[k + 12 * j];
      state_and_control_ref[k + 12] = 0;
      p_feet_and_gait_state[k] = p_feet[k + 12 * j];
    }
    for (int k = 0; k < 4; k++) {
      p_feet_and_gait_state[k + 12] = gait_data[k + 4 * j];
    }
    double x_drag = 0;
    p_feet_and_gait_state[16] = x_drag;

    // 损失函数的模型参数，有期望状态和参考输入，但是在损失函数是对输入损失正则化，
    // 故传入的参考输入为0，官网说acados自带热启动功能，会储存上一次的优化结果，
    // 所以不需要给参考输入速度也很快。
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, j, "yref",
                           state_and_control_ref);
    srbd_acados_update_params(acados_ocp_capsule, j, p_feet_and_gait_state, 17);
  }

  // 最后时刻的期望状态单独赋值，因为这一时刻没有参考输入
  double terminal_state[12];
  for (int k = 0; k < 12; k++) {
    terminal_state[k] = desired_state[k + 12 * horizons];
  }
  ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, horizons, "yref",
                         terminal_state);
}

// 求解非线性优化问题并返回最优状态和输入轨迹
void nmpc_caller::get_solution(VectorXd &optimal_state,
                               VectorXd &optimal_control) {

  status = srbd_acados_solve(acados_ocp_capsule);

  // 判断一下是否有解，无解的时候要保留上次的结果，直至有解的时候才能更新
  if (status != ACADOS_SUCCESS) {
    printf("acados_solve() failed with status %d.\n", status);
    optimal_state = last_optimal_state;
    optimal_control = last_optimal_control;
  } else {
    // get the optimized solution
    double _optimal_state[12];
    double _optimal_control[12];

    for (int i = 0; i < horizons; i++) {
      ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i + 1, "x",
                      _optimal_state);
      ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "u", _optimal_control);

      copy_Array_to_Eigen(last_optimal_state, _optimal_state, 12, i * 12, 0);
      copy_Array_to_Eigen(last_optimal_control, _optimal_control, 12, i * 12,
                          0);
    }
    optimal_state = last_optimal_state;
    optimal_control = last_optimal_control;
  }
}