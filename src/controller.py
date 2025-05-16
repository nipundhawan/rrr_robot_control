import numpy as np

class ComputedTorqueController:
    def __init__(self, robot, Kp=100.0, Kd=20.0):
        self.robot = robot
        self.Kp    = Kp
        self.Kd    = Kd

    def compute(self, q, q_dot, x_des, xdot_des):
        # 1) current Cartesian pos & vel
        x    = self.robot.forward_kinematics(q)
        J    = self.robot.jacobian(q)
        x_dot= J @ q_dot

        # 2) PD in Cartesian space
        e     = x_des    - x
        e_dot = xdot_des - x_dot
        Fcart = self.Kp*e + self.Kd*e_dot

        # 3) map to joint torques
        tau = J.T @ Fcart + self.robot.coriolis(q, q_dot) + self.robot.gravity(q)

        # 4) solve for q̈:  M q̈ + C + G = τ
        Mq    = self.robot.mass_matrix(q)
        q_ddot= np.linalg.solve(Mq, tau - self.robot.coriolis(q, q_dot) - self.robot.gravity(q))

        return q_ddot
