from localization_system import LocalizationSystem


def main():
    loc = LocalizationSystem()

    encoder_data = [
        {"x": 1.0, "y": 0.0, "theta": 0.0},
        {"x": 2.0, "y": 0.0, "theta": 0.0},
        {"x": 3.0, "y": 0.0, "theta": 0.0},
        {"x": 4.0, "y": 0.0, "theta": 0.0}
    ]

    camera_data = [
        None,
        {"x": 1.8, "y": 0.1, "theta": 0.0, "valid": True},
        {"x": 10.0, "y": 10.0, "theta": 0.0, "valid": True},
        {"x": 3.9, "y": 0.05, "theta": 0.01, "valid": True}
    ]

    for i in range(len(encoder_data)):
        state = loc.step(encoder_data[i], camera_data[i])
        print(f"Pas {i + 1}: {state}")


if __name__ == "__main__":
    main()