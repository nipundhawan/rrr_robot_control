import numpy as np
from src.robot import RRRRobot
from src.controller import ComputedTorqueController


def test_pd_output_finite():
    robot = RRRRobot()
    ctrl = ComputedTorqueController(robot)
    q = np.zeros(3)
    qdot = np.zeros(3)
    x_des = np.array([3.0, 0.0])
    xdot_des = np.zeros(2)
    qdd = ctrl.compute(q, qdot, x_des, xdot_des)
    assert np.all(np.isfinite(qdd))
