
#include "nmpc_caller.h"

int main() {

  int horizons = 26;

  double dt = 0.002;
  int iteration_between_mpc = 5;
  double dtmpc = dt * iteration_between_mpc;

  int iterations = 1 / dtmpc;

  VectorXd time_record(iterations - 1);

  // contact point
  VectorXd p_feet(12 * horizons);
  for (int i = 0; i < horizons; i++) {
    p_feet.block(12 * i, 0, 12, 1) << 0.209426, -0.109557, -0.003, 0.214422,
        0.114664, -0.003, -0.165504, -0.113441, -0.003, -0.169422, 0.11112,
        -0.003;
  }

  double vel_x = 0.1;
  double vel_y = 0.05;
  double vel_z = 0.1;
  double vel_roll = 0.;
  double vel_pitch = 0;
  double vel_yaw = 0.5;
  VectorXd current_state;
  current_state.setZero(12);
  current_state << 0, 0, 0, vel_x, vel_y, vel_z, 0, 0, 0, vel_roll, vel_pitch,
      vel_yaw;

  VectorXd gait_data(4 * horizons);
  for (int i = 0; i < horizons; i++) {
    gait_data.block(4 * i, 0, 4, 1) << 1, 1, 1, 1;
  }

  nmpc_caller nmpc_caller;

  // closed loop simulation
  for (int i = 0; i < iterations; i++) {
    cout << "************** time = " << (i + 1) * dtmpc << " s **************\n"
         << endl;
    auto t_start = chrono::high_resolution_clock::now();
    cout << "current_state\n" << current_state.transpose() << "\n" << endl;

    VectorXd desired_state(12 * horizons);
    for (int j = 0; j < horizons; j++) {
      desired_state.block(j * 12, 0, 12, 1) << vel_x * (1 + i + j) * dtmpc,
          vel_y * (1 + i + j) * dtmpc, vel_z * (1 + i + j) * dtmpc, vel_x,
          vel_y, vel_z, vel_roll * (1 + i + j) * dtmpc,
          vel_pitch * (1 + i + j) * dtmpc, vel_yaw * (1 + i + j) * dtmpc,
          vel_roll, vel_pitch, vel_yaw;
    }
    nmpc_caller.nmpc_setup(current_state, p_feet, gait_data, desired_state);

    VectorXd optimal_state(12 * horizons), optimal_control(12 * horizons);
    nmpc_caller.get_solution(optimal_state, optimal_control);
    current_state = optimal_state.block(0, 0, 12, 1);

    auto t_end = chrono::high_resolution_clock::now();
    double elapsed_time_ms =
        chrono::duration<double, milli>(t_end - t_start).count();
    if (i != 0) {
      time_record(i - 1) = elapsed_time_ms;
    }

    cout << "desired_state\n"
         << desired_state.block(0, 0, 12, 1).transpose() << "\n"
         << endl;
    cout << "optimal_state\n"
         << optimal_state.block(0, 0, 12, 1).transpose() << "\n"
         << endl;
    cout << "optimal_control\n"
         << optimal_control.block(0, 0, 12, 1).transpose() << "\n"
         << endl;
  }

  cout << "average estimation time = " << time_record.mean() / 1000 << "s"
       << endl;
  cout << "max estimation time = " << time_record.maxCoeff() / 1000 << "s"
       << endl;
  cout << "min estimation time = " << time_record.minCoeff() / 1000 << "s"
       << endl;
}