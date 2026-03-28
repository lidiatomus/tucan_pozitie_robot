from localization_system import LocalizationSystem


def main():
    loc = LocalizationSystem()

    odom_data = [
        {"v": 1.0, "omega": 0.0, "dt": 1.0},
        {"v": 1.0, "omega": 0.0, "dt": 1.0},
        {"v": 1.0, "omega": 0.0, "dt": 1.0},
        {"v": 1.0, "omega": 0.0, "dt": 1.0}
    ]

    measurements = [
        None,
        {"x": 1.8, "y": 0.1, "theta": 0.0, "valid": True},
        {"x": 10.0, "y": 10.0, "theta": 0.0, "valid": True},
        {"x": 3.75, "y": 0.12, "theta": 0.01, "valid": True}
    ]

    for i in range(len(odom_data)):
        state = loc.step(odom_data[i], measurements[i])
        print(f"Pas {i + 1}: {state}")


if __name__ == "__main__":
    main()