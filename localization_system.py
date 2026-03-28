import numpy as np
from ekf import EKFLocalization
from math_utils import normalize_angle


def is_measurement_reasonable(state, measurement, max_dist=0.5, max_angle=np.deg2rad(45)):
    dx = measurement["x"] - state["x"]
    dy = measurement["y"] - state["y"]
    dist = np.hypot(dx, dy)

    dtheta = normalize_angle(measurement["theta"] - state["theta"])

    return dist < max_dist and abs(dtheta) < max_angle


class LocalizationSystem:
    def __init__(self):
        self.ekf = EKFLocalization()

    def step(self, odom, measurement=None):
        if not isinstance(odom, dict):
            raise TypeError("odom trebuie sa fie dict")

        required_odom_keys = ["v", "omega", "dt"]
        for key in required_odom_keys:
            if key not in odom:
                raise KeyError(f"Lipseste cheia '{key}' din odom")

        v = float(odom["v"])
        omega = float(odom["omega"])
        dt = float(odom["dt"])

        if dt <= 0:
            raise ValueError("dt trebuie sa fie > 0")

        self.ekf.predict(v, omega, dt)

        measurement_used = False

        if measurement is not None:
            if not isinstance(measurement, dict):
                raise TypeError("measurement trebuie sa fie dict sau None")

            if measurement.get("valid", False):
                required_meas_keys = ["x", "y", "theta"]
                for key in required_meas_keys:
                    if key not in measurement:
                        raise KeyError(f"Lipseste cheia '{key}' din measurement")

                current_state = self.ekf.get_state()

                if is_measurement_reasonable(current_state, measurement):
                    z = [
                        float(measurement["x"]),
                        float(measurement["y"]),
                        float(measurement["theta"])
                    ]
                    self.ekf.update(z)
                    measurement_used = True

        state = self.ekf.get_state()
        state["measurement_used"] = measurement_used
        return state