import numpy as np
import casadi as ca
from acados_template import AcadosModel


# rpyz转为旋转矩阵
def rpy2rot(rpy):
    Rx = ca.SX.eye(3)
    Ry = ca.SX.eye(3)
    Rz = ca.SX.eye(3)
    R = ca.SX.eye(3)

    Rz[0, 0] = ca.cos(rpy[2])
    Rz[0, 1] = -ca.sin(rpy[2])
    Rz[1, 0] = ca.sin(rpy[2])
    Rz[1, 1] = ca.cos(rpy[2])

    Ry[0, 0] = ca.cos(rpy[1])
    Ry[0, 2] = ca.sin(rpy[1])
    Ry[2, 0] = -ca.sin(rpy[1])
    Ry[2, 2] = ca.cos(rpy[1])

    Rx[1, 1] = ca.cos(rpy[0])
    Rx[1, 2] = -ca.sin(rpy[0])
    Rx[2, 1] = ca.sin(rpy[0])
    Rx[2, 2] = ca.cos(rpy[0])

    R = Rz @ Ry @ Rx

    return R


# 角速度转为欧拉角速度
def omega2rpydot(rpy, omega):
    rpy_dot = ca.SX(3, 1)
    trans_mat = ca.SX.eye(3)

    trans_mat[0, 0] = ca.cos(rpy[2]) / ca.cos(rpy[1])
    trans_mat[0, 1] = ca.sin(rpy[2]) / ca.cos(rpy[1])
    trans_mat[0, 2] = 0
    trans_mat[1, 0] = -ca.sin(rpy[2])
    trans_mat[1, 1] = ca.cos(rpy[2])
    trans_mat[1, 2] = 0
    trans_mat[2, 0] = ca.cos(rpy[2]) * ca.tan(rpy[1])
    trans_mat[2, 1] = ca.sin(rpy[2]) * ca.tan(rpy[1])
    trans_mat[2, 2] = 1

    rpy_dot = trans_mat @ omega

    return rpy_dot


# 求向量的反对称矩阵
def vec2mat(vector):
    mat = ca.SX.zeros(3, 3)

    mat[0, 1] = -vector[2]
    mat[0, 2] = vector[1]
    mat[1, 0] = vector[2]
    mat[1, 2] = -vector[0]
    mat[2, 0] = -vector[1]
    mat[2, 1] = vector[0]

    return mat


# 单刚体动力学SRBD模型
class SRBD_Model(object):
    def __init__(
        self,
    ):
        model = AcadosModel()
        model.name = "srbd"

        inertia = np.eye(3)
        inertia[0, 0] = 0.07
        inertia[1, 1] = 0.26
        inertia[2, 2] = 0.242
        mass = 8.9
        g = [0, 0, 9.81]

        state_vars = ca.SX.sym("state_vars", 12)
        control_vars = ca.SX.sym("control_vars", 12)

        feet_state = ca.SX.sym("feet_state", 12)
        gait_data = ca.SX.sym("gait_data", 4)
        x_drag = ca.SX.sym("x_drag", 1)

        rotation_mat = rpy2rot(state_vars[6:9])
        inertia_inv = ca.inv(rotation_mat @ inertia @ rotation_mat.T)

        tau = (
            ca.cross(feet_state[0:3] - state_vars[0:3], control_vars[0:3], -1)
            + ca.cross(feet_state[3:6] - state_vars[0:3], control_vars[3:6], -1)
            + ca.cross(feet_state[6:9] - state_vars[0:3], control_vars[6:9], -1)
            + ca.cross(feet_state[9:12] - state_vars[0:3], control_vars[9:12], -1)
        )
        f = (
            control_vars[0:3]
            + control_vars[3:6]
            + control_vars[6:9]
            + control_vars[9:12]
        )

        # 状态空间
        state_space = ca.vertcat(
            state_vars[3:6],
            f / mass - g,
            omega2rpydot(state_vars[6:9], state_vars[9:12]),
            inertia_inv
            @ (tau - vec2mat(state_vars[9:12]) @ (inertia @ state_vars[9:12])),
        )
        state_space[5] += x_drag

        state_vars_dot = ca.SX.sym("state_vars_dot", 12)
        f_impl = state_space

        model.f_expl_expr = f_impl
        model.f_impl_expr = state_vars_dot - f_impl
        model.x = state_vars
        model.xdot = state_vars_dot
        model.u = control_vars
        # 参数由足端位置和步态信息构成
        model.p = ca.vertcat(feet_state, gait_data, x_drag)

        # 足底力约束
        mu = 0.4
        nonlinear_constraint = ca.SX.sym("nonlinear_constraint", 20)
        for i in range(4):
            nonlinear_constraint[i * 5 + 0] = (
                control_vars[i * 3 + 0] - mu * control_vars[i * 3 + 2]
            )
            nonlinear_constraint[i * 5 + 1] = (
                control_vars[i * 3 + 1] - mu * control_vars[i * 3 + 2]
            )
            nonlinear_constraint[i * 5 + 2] = (
                control_vars[i * 3 + 0] + mu * control_vars[i * 3 + 2]
            )
            nonlinear_constraint[i * 5 + 3] = (
                control_vars[i * 3 + 1] + mu * control_vars[i * 3 + 2]
            )
            nonlinear_constraint[i * 5 + 4] = control_vars[i * 3 + 2]

        model.con_h_expr = nonlinear_constraint

        constraint = ca.types.SimpleNamespace()
        constraint.lower_bound = np.array(
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        )
        constraint.upper_bound = np.array(
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        )

        self.model = model
        self.constraint = constraint
