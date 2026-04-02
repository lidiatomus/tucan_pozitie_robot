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

    def step(self, encoder_pose, camera_measurement=None):
        if not isinstance(encoder_pose, dict):
            raise TypeError("encoder_pose trebuie sa fie dict")

        required_encoder_keys = ["x", "y", "theta"]
        for key in required_encoder_keys:
            if key not in encoder_pose:
                raise KeyError(f"Lipseste cheia '{key}' din encoder_pose")

        self.ekf.predict_from_encoder_pose(encoder_pose)

        measurement_used = False

        if camera_measurement is not None:
            if not isinstance(camera_measurement, dict):
                raise TypeError("camera_measurement trebuie sa fie dict sau None")

            if camera_measurement.get("valid", False):
                required_meas_keys = ["x", "y", "theta"]
                for key in required_meas_keys:
                    if key not in camera_measurement:
                        raise KeyError(f"Lipseste cheia '{key}' din camera_measurement")

                current_state = self.ekf.get_state()

                if is_measurement_reasonable(current_state, camera_measurement):
                    z = [
                        float(camera_measurement["x"]),
                        float(camera_measurement["y"]),
                        float(camera_measurement["theta"])
                    ]
                    self.ekf.update(z)
                    measurement_used = True

        state = self.ekf.get_state()
        state["measurement_used"] = measurement_used
        return state