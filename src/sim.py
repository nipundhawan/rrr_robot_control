import numpy as np


class Target:
    """
    Sinusoidal target generator:
    x fixed at 2L, y = L·sin(2πf t), measured at 30 Hz.
    """

    def __init__(self, L=1.0, f=5.0):
        self.L = L
        self.f = f
        self.t = 0.0

    def step(self, dt):
        """
        dt: time step (1/30)
        returns: (position [x,y], velocity [vx,vy])
        """
        self.t += dt
        x = 2 * self.L
        y = self.L * np.sin(2 * np.pi * self.f * self.t)
        y_dot = self.L * (2 * np.pi * self.f) * np.cos(2 * np.pi * self.f * self.t)
        return np.array([x, y]), np.array([0.0, y_dot])
