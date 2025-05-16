import numpy as np
from src.robot import RRRRobot

def test_forward_kinematics_zero():
    robot = RRRRobot(L=1.0)
    # all zeros → arm extended length 3 along x
    assert np.allclose(robot.forward_kinematics([0, 0, 0]), [3.0, 0.0])
