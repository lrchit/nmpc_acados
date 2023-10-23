from srbd_model import SRBD_Model
from srbd_ocp_setting import SRBD_Optimizer
import numpy as np
import timeit

if __name__ == "__main__":
    horizon = 26
    dt = 0.002
    iteration_between_mpc = 5
    dtmpc = dt * iteration_between_mpc

    iterations = int(1 / dtmpc)

    _t_horizon = dtmpc * horizon
    _n_nodes = horizon

    srbd_model = SRBD_Model()
    opt = SRBD_Optimizer(
        _model=srbd_model.model,
        _constraint=srbd_model.constraint,
        t_horizon=_t_horizon,
        n_nodes=_n_nodes,
        debug_state=1,
    )
