import numpy as np
from math_utils import normalize_angle


class EKFLocalization:
    def __init__(self, initial_state=None, initial_cov=None):
        if initial_state is None:
            initial_state = np.zeros((3, 1))

        if initial_cov is None:
            initial_cov = np.eye(3) * 0.1

        self.x = initial_state.astype(float).reshape(3, 1)
        self.P = initial_cov.astype(float)

        # Zgomotul modelului de mișcare
        self.Q = np.diag([0.02, 0.02, np.deg2rad(3.0)]) ** 2

        # Zgomotul măsurătorii de la cameră
        self.R = np.diag([0.05, 0.05, np.deg2rad(5.0)]) ** 2

    def predict(self, v, omega, dt):
        theta = self.x[2, 0]

        self.x[0, 0] += v * np.cos(theta) * dt
        self.x[1, 0] += v * np.sin(theta) * dt
        self.x[2, 0] += omega * dt
        self.x[2, 0] = normalize_angle(self.x[2, 0])

        F = np.array([
            [1.0, 0.0, -v * np.sin(theta) * dt],
            [0.0, 1.0,  v * np.cos(theta) * dt],
            [0.0, 0.0, 1.0]
        ])

        self.P = F @ self.P @ F.T + self.Q

    def get_state(self):
        return {
            "x": float(self.x[0, 0]),
            "y": float(self.x[1, 0]),
            "theta": float(self.x[2, 0])
        }
    def update(self, z):
        z = np.asarray(z, dtype=float).reshape(3, 1)

        H = np.eye(3)

        y = z - H @ self.x
        y[2, 0] = normalize_angle(y[2, 0])

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2, 0] = normalize_angle(self.x[2, 0])

        I = np.eye(3)
        self.P = (I - K @ H) @ self.P
    def set_process_noise(self, qx, qy, qtheta_deg):
        self.Q = np.diag([qx, qy, np.deg2rad(qtheta_deg)]) ** 2

    def set_measurement_noise(self, rx, ry, rtheta_deg):
        self.R = np.diag([rx, ry, np.deg2rad(rtheta_deg)]) ** 2