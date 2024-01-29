import os
import sys
import shutil
import errno
import numpy as np
import scipy.linalg
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSim, AcadosSimSolver


def safe_mkdir_recursive(directory, overwrite=False):
    if not os.path.exists(directory):
        try:
            os.makedirs(directory)
        except OSError as exc:
            if exc.errno == errno.EEXIST and os.path.isdir(directory):
                pass
            else:
                raise
    else:
        if overwrite:
            try:
                shutil.rmtree(directory)
            except:
                print("Error while removing directory {}".format(directory))


class SRBD_Optimizer(object):

    def __init__(self, _model, _constraint, t_horizon, n_nodes, debug_state):
        model = _model
        self.T = t_horizon
        self.N = n_nodes

        # 状态维数，输入维数，参数维数（足端位置和步态信息）
        nx = model.x.size()[0]
        self.nx = nx
        nu = model.u.size()[0]
        self.nu = nu
        ny = nx + nu
        nparams = model.p.size()[0]
        n_params = nparams

        # OCP
        ocp = AcadosOcp()
        # 加上这个路径好像会算的快一点
        acados_source_path = os.environ["ACADOS_SOURCE_DIR"]
        sys.path.insert(0, acados_source_path)
        ocp.acados_include_path = acados_source_path + "/include"
        ocp.acados_lib_path = acados_source_path + "/lib"

        ocp.model = model

        # 指定每次mpc预测的步数和时间跨度
        ocp.dims.N = self.N
        ocp.solver_options.tf = self.T

        # 指定参数维数
        ocp.dims.np = n_params
        ocp.parameter_values = np.zeros(n_params)

        # 损失函数，Q，R分别为状态和输入损失参数
        Q = np.zeros((self.nx, self.nx))
        Q[0, 0] = 2
        Q[1, 1] = 2
        Q[2, 2] = 40
        Q[3, 3] = 0.2
        Q[4, 4] = 0.2
        Q[5, 5] = 0.2
        Q[6, 6] = 0.25
        Q[7, 7] = 0.25
        Q[8, 8] = 10
        Q[9, 9] = 0
        Q[10, 10] = 0
        Q[11, 11] = 0.3
        R = np.zeros((self.nx, self.nx))
        R[0, 0] = 0.00005
        R[1, 1] = 0.00005
        R[2, 2] = 0.00005
        R[3, 3] = 0.00005
        R[4, 4] = 0.00005
        R[5, 5] = 0.00005
        R[6, 6] = 0.00005
        R[7, 7] = 0.00005
        R[8, 8] = 0.00005
        R[9, 9] = 0.00005
        R[10, 10] = 0.00005
        R[11, 11] = 0.00005
        alpha = 2
        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.cost_type_e = "LINEAR_LS"
        ocp.cost.W = scipy.linalg.block_diag(Q, R)
        # horizon终点权重加大，但是效果不好
        ocp.cost.W_e = Q * alpha
        ocp.cost.Vx = np.zeros((ny, nx))
        ocp.cost.Vx[:nx, :nx] = np.eye(nx)
        ocp.cost.Vu = np.zeros((ny, nu))
        ocp.cost.Vu[-nu:, -nu:] = np.eye(nu)
        ocp.cost.Vx_e = np.eye(nx)

        # 约束函数的上下界
        ocp.constraints.lh = _constraint.lower_bound
        ocp.constraints.uh = _constraint.upper_bound

        # 下面这段可能只是用来指定损失函数中状态和输入的维数
        x_ref = np.zeros(nx)
        u_ref = np.zeros(nu)
        ocp.constraints.x0 = x_ref
        ocp.cost.yref = np.concatenate((x_ref, u_ref))
        ocp.cost.yref_e = x_ref

        # solver options，使用HPIPM求解，用高斯牛顿法求Hessian矩阵
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.ext_fun_compile_flags = "-O3"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.qp_solver_warm_start = 0
        ocp.solver_options.qp_solver_ric_alg = 0
        # ocp.solver_options.qp_solver_cond_ric_alg = 0
        # 使用龙格库塔法，SQP，HPIPM
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.nlp_solver_max_iter = 1000
        ocp.solver_options.hpipm_mode = "SPEED_ABS"
        ocp.solver_options.nlp_solver_tol_stat = 1e-6
        ocp.solver_options.nlp_solver_tol_eq = 1e-8
        ocp.solver_options.nlp_solver_tol_ineq = 1e-8
        ocp.solver_options.nlp_solver_tol_comp = 1e-8

        # 生成ocp
        json_file = os.path.join("./" + model.name + "_acados_ocp.json")
        if debug_state:
            self.solver = AcadosOcpSolver(ocp, json_file=json_file)
        else:
            self.solver = AcadosOcpSolver(ocp,
                                          json_file=json_file,
                                          build=False,
                                          generate=False,
                                          verbose=False)

    #     # 用自带的积分器进行仿真
    #     sim = AcadosSim()
    #     sim.acados_include_path = acados_source_path + '/include'
    #     sim.acados_lib_path = acados_source_path + '/lib'
    #     # set model
    #     sim.model = model

    #     # 可以加入扰动
    #     # To be added later

    #     # solver options，也使用龙阁库塔法，但是结果和优化不一致，不知道为什么
    #     sim.solver_options.integrator_type = "ERK"

    #     # set prediction horizon
    #     sim.parameter_values = np.zeros(n_params)
    #     sim.solver_options.T = self.T / self.N
    #     sim.solver_options.sim_method_num_stages = 4

    #     # 生成积分器
    #     json_file = os.path.join("./" + model.name + "_acados_sim.json")
    #     if debug_state:
    #         self.integrator = AcadosSimSolver(sim, json_file=json_file)
    #     else:
    #         self.integrator = AcadosSimSolver(sim,
    #                                           json_file=json_file,
    #                                           generate=False,
    #                                           build=False,
    #                                           verbose=False)

    # def sovle_and_simulate(self, _current_state, _state_ref, _control_ref,
    #                        _p_feet, _gait_data):

    #     # 给ocp传入当前状态，足端位置和步态信息
    #     self.solver.set(self.N, "y_ref", _state_ref[self.N, :])
    #     for j in range(self.N):
    #         _p_feet_and_gait_state = np.concatenate(
    #             (_p_feet, _gait_data[j, :]))
    #         self.solver.set(j, "p", _p_feet_and_gait_state)
    #         state_and_control_ref = np.concatenate(
    #             (_state_ref[j, :], np.zeros(12)))
    #         self.solver.set(j, "y_ref", state_and_control_ref)

    #     # solve ocp
    #     # set initial (stage 0)，因为没有将dynamics作为约束，
    #     # 故需要将第一步的状态约束为初始状态，否则没有初始状态
    #     self.solver.set(0, "lbx", _current_state)
    #     self.solver.set(0, "ubx", _current_state)

    #     # 求解ocp并得到返回值
    #     status = self.solver.solve()
    #     # 有解则update优化输入
    #     if status == 0:
    #         for j in range(self.N):
    #             _control_ref[j, :] = self.solver.get(j, "u")
    #     # 无解也要将上一次的解保留，否则会出问题，这里无解直接中断，便于分析问题
    #     else:
    #         # print('acados acados_ocp_solver returned status {}. Using last solution.'.format(status))
    #         raise Exception(
    #             "acados acados_ocp_solver returned status {}. terminated.".
    #             format(status))

    #     # simulate system
    #     # 用积分器仿真，输入当前状态，第一步的优化输入以及参数，积分得到下一步状态
    #     self.integrator.set("x", _current_state)
    #     self.integrator.set("u", _control_ref[0, :])
    #     _p_feet_and_gait_state = np.concatenate((_p_feet, _gait_data[0, :]))
    #     self.integrator.set("p", _p_feet_and_gait_state)
    #     # 积分器求解
    #     status_s = self.integrator.solve()
    #     # 有解则将积分结果更新状态
    #     if status_s == 0:
    #         # update
    #         # _current_state = self.integrator.get("x")
    #         _current_state = self.solver.get(1, 'x')
    #     else:
    #         _current_state = self.solver.get(1, "x")
    #         # print('acados integrator returned status {}. Using optimal trajectroy.'.format(status))
    #         raise Exception(
    #             "acados integrator returned status {}. terminated.".format(
    #                 status))

    #     # 返回更新状态和优化输入
    #     return _current_state, _control_ref
