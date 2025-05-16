import numpy as np


class RRRRobot:
    def __init__(self, L=1.0, m1=1.0, m2=1.0, m3=1.0, g=9.81):
        # link lengths
        self.L1 = self.L2 = self.L3 = L
        # link masses
        self.m1, self.m2, self.m3 = m1, m2, m3
        self.g = g

    def forward_kinematics(self, q):
        q1, q2, q3 = q
        x1 = self.L1 * np.cos(q1)
        y1 = self.L1 * np.sin(q1)
        x2 = x1 + self.L2 * np.cos(q1 + q2)
        y2 = y1 + self.L2 * np.sin(q1 + q2)
        x3 = x2 + self.L3 * np.cos(q1 + q2 + q3)
        y3 = y2 + self.L3 * np.sin(q1 + q2 + q3)
        return np.array([x3, y3])

    def jacobian(self, q):
        q1, q2, q3 = q
        θ12 = q1 + q2
        θ123 = θ12 + q3
        J = np.zeros((2, 3))
        J[0, 0] = -self.L1 * np.sin(q1) - self.L2 * np.sin(θ12) - self.L3 * np.sin(θ123)
        J[0, 1] = -self.L2 * np.sin(θ12) - self.L3 * np.sin(θ123)
        J[0, 2] = -self.L3 * np.sin(θ123)
        J[1, 0] = self.L1 * np.cos(q1) + self.L2 * np.cos(θ12) + self.L3 * np.cos(θ123)
        J[1, 1] = self.L2 * np.cos(θ12) + self.L3 * np.cos(θ123)
        J[1, 2] = self.L3 * np.cos(θ123)
        return J

    def mass_matrix(self, q):
        """3×3 joint-space inertia matrix M(q)."""
        q1, q2, q3 = q
        # shorthand
        L1, L2, L3 = self.L1, self.L2, self.L3
        m1, m2, m3 = self.m1, self.m2, self.m3
        # compute elements per standard RRR model
        I11 = (
            m1 * L1**2
            + m2 * (L1**2 + L2**2 + 2 * L1 * L2 * np.cos(q2))
            + m3
            * (
                L1**2
                + L2**2
                + L3**2
                + 2 * L1 * L2 * np.cos(q2)
                + 2 * L2 * L3 * np.cos(q3)
                + 2 * L1 * L3 * np.cos(q2 + q3)
            )
        )
        I12 = m2 * (L2**2 + L1 * L2 * np.cos(q2)) + m3 * (
            L2**2
            + L3**2
            + L1 * L2 * np.cos(q2)
            + 2 * L2 * L3 * np.cos(q3)
            + L1 * L3 * np.cos(q2 + q3)
        )
        I13 = m3 * (L3**2 + L2 * L3 * np.cos(q3) + L1 * L3 * np.cos(q2 + q3))
        I22 = m2 * L2**2 + m3 * (L2**2 + L3**2 + 2 * L2 * L3 * np.cos(q3))
        I23 = m3 * (L3**2 + L2 * L3 * np.cos(q3))
        I33 = m3 * L3**2
        M = np.array([[I11, I12, I13], [I12, I22, I23], [I13, I23, I33]])
        return M

    def coriolis(self, q, q_dot):
        """3-vector C(q, q_dot) collecting Coriolis+centrifugal terms."""
        # For brevity, use Christoffel symbols via numeric differentiation:
        M = self.mass_matrix(q)
        C = np.zeros(3)
        eps = 1e-6
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    # ∂M_ij/∂q_k, etc.
                    dqk = np.zeros(3)
                    dqk[k] = eps
                    dM_ij = (self.mass_matrix(q + dqk)[i, j] - M[i, j]) / eps
                    dM_ik = (self.mass_matrix(q + dqk)[i, k] - M[i, k]) / eps
                    dM_jk = (self.mass_matrix(q + dqk)[j, k] - M[j, k]) / eps
                    C[i] += 0.5 * (dM_ij + dM_ik - dM_jk) * q_dot[j] * q_dot[k]
        return C

    def gravity(self, q):
        """3-vector of gravity torques g(q)."""
        q1, q2, q3 = q
        L1, L2, L3 = self.L1, self.L2, self.L3
        m1, m2, m3 = self.m1, self.m2, self.m3
        g = self.g
        # potential energy partials:
        g1 = g * (
            m1 * (L1 / 2) * np.cos(q1)
            + m2 * (L1 * np.cos(q1) + (L2 / 2) * np.cos(q1 + q2))
            + m3
            * (L1 * np.cos(q1) + L2 * np.cos(q1 + q2) + (L3 / 2) * np.cos(q1 + q2 + q3))
        )
        g2 = g * (
            m2 * ((L2 / 2) * np.cos(q1 + q2))
            + m3 * (L2 * np.cos(q1 + q2) + (L3 / 2) * np.cos(q1 + q2 + q3))
        )
        g3 = g * (m3 * ((L3 / 2) * np.cos(q1 + q2 + q3)))
        return np.array([g1, g2, g3])
