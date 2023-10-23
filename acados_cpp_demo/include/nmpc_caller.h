
#ifndef NMPC_CALLER_H
#define NMPC_CALLER_H

#include <chrono>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"

#include "acados_solver_srbd.h"

using namespace std;
using namespace Eigen;

class nmpc_caller {
public:
  nmpc_caller();
  ~nmpc_caller();

  void nmpc_setup(VectorXd _current_state, VectorXd _p_feet,
                  VectorXd _gait_data, VectorXd _desired_state);
  void get_solution(VectorXd &optimal_state, VectorXd &optimal_control);

  void copy_Array_to_Eigen(VectorXd &target, double *source, int len,
                           int startIndex_eigen, int startIndex_array);
  void copy_Eigen_to_Array(double *target, VectorXd &source, int len,
                           int startIndex_array, int startIndex_eigen);

private:
  int status; // acados operation state
  int horizons;
  int nx;
  int nu;

  srbd_solver_capsule *acados_ocp_capsule;
  ocp_nlp_config *nlp_config;
  ocp_nlp_dims *nlp_dims;
  ocp_nlp_in *nlp_in;
  ocp_nlp_out *nlp_out;

  VectorXd last_optimal_state;
  VectorXd last_optimal_control;
};

#endif // NMPC_CALLER_H
