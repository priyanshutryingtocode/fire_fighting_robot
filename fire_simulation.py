"""Simulate the robot scanning, aligning, approaching, and avoiding obstacles.

This is a hardware-free scenario runner. It uses the real Navigator policy,
but supplies scripted detector results and sensor readings. No camera, Arduino,
motors, pump, or model weights are required.

Run with: python fire_simulation.py
"""
import argparse
import time

from navigation import Navigator, SensorData


def detection(confirmed=False, direction="NONE"):
    """Build a detector-shaped result for one simulated camera frame."""
    strongest = None
    if direction != "NONE":
        strongest = {
            "class_name": "FIRE",
            "confidence": 0.91,
            "direction": direction,
        }
    return {
        "detections": [strongest] if strongest else [],
        "strongest": strongest,
        "confirmed": confirmed,
        "consecutive_count": 4 if confirmed else 0,
        "fire_direction": direction,
    }


def run_simulation(delay=0.5):
    """Run a short scenario and print each state/action decision."""
    navigator = Navigator()
    # The robot scans until fire is confirmed, aligns it to the center, then
    # approaches. Obstacle handling takes priority over approaching the fire.
    scenario = [
        ("Starting idle", False, detection(), SensorData(120, 100)),
        ("Scanning with no confirmed fire", True, detection(), SensorData(120, 100)),
        ("Fire seen on left, not yet confirmed", True,
         detection(False, "LEFT"), SensorData(120, 100)),
        ("Fire confirmed on left", True,
         detection(True, "LEFT"), SensorData(120, 100)),
        ("Robot aligned; begin approaching centered fire", True,
         detection(True, "CENTER"), SensorData(120, 100)),
        ("Obstacle detected while approaching", True,
         detection(True, "CENTER"), SensorData(18, 100)),
        ("Scenario stopped", False, detection(), SensorData(120, 100)),
    ]

    print("Fire response simulation (scripted inputs; no hardware control)\n")
    last_action = None
    for tick, (event, start, fire_result, sensors) in enumerate(scenario, 1):
        decision = navigator.update(fire_result, sensors, start=start)
        changed = " (changed)" if decision.action != last_action else ""
        print(
            f"{tick:02d}. {event}\n"
            f"    fire={fire_result['fire_direction']} "
            f"confirmed={fire_result['confirmed']} "
            f"front={sensors.front_distance:g}cm rear={sensors.rear_distance:g}cm\n"
            f"    state={decision.state.name} action={decision.action.value}{changed}\n"
            f"    reason={decision.reason}"
        )
        last_action = decision.action
        if delay:
            time.sleep(delay)

    print("\nSimulation complete. Suppression and arm actuation are not implemented.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--delay", type=float, default=0.5,
                        help="Seconds between simulated steps (default: 0.5; use 0 for instant)")
    args = parser.parse_args()
    if args.delay < 0:
        parser.error("--delay must be zero or greater")
    run_simulation(args.delay)


if __name__ == "__main__":
    main()
