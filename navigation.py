"""High-level, hardware-independent navigation state machine."""
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional


class RobotState(Enum):
    IDLE = auto()
    SCAN_FIRE = auto()
    AVOID_OBSTACLE = auto()
    ALIGN_FIRE = auto()
    APPROACH_FIRE = auto()
    SAFE_STOP = auto()


class Action(Enum):
    STOP = "STOP"
    FORWARD = "FORWARD"
    REVERSE = "REVERSE"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    SCAN_360 = "SCAN_360"


@dataclass
class SensorData:
    front_distance: Optional[float] = None
    rear_distance: Optional[float] = None
    flame_detected: Optional[bool] = None
    pir_detected: Optional[bool] = None


@dataclass
class NavigationDecision:
    state: RobotState
    action: Action
    reason: str


class Navigator:
    """
    Conservative prototype controller.

    It never turns automatically because the current hardware has no left/right
    range measurement. A reverse action is only requested when rear clearance
    is known. The Arduino firmware must independently enforce its watchdog and
    distance stop rules.
    """
    def __init__(self, front_stop_cm=25.0, rear_clear_cm=20.0):
        self.state = RobotState.IDLE
        self.front_stop_cm = front_stop_cm
        self.rear_clear_cm = rear_clear_cm

    def update(self, fire_result, sensors: SensorData, start=False):
        if not start:
            self.state = RobotState.IDLE
            return NavigationDecision(self.state, Action.STOP, "Waiting for start")

        # Missing/invalid front reading means no permission to move forward.
        if sensors.front_distance is None or sensors.front_distance <= 0:
            self.state = RobotState.SAFE_STOP
            return NavigationDecision(self.state, Action.STOP, "Front range unavailable")

        # Obstacle handling takes precedence over fire-seeking.
        if sensors.front_distance <= self.front_stop_cm:
            self.state = RobotState.AVOID_OBSTACLE
            if (sensors.rear_distance is not None
                    and sensors.rear_distance > self.rear_clear_cm):
                return NavigationDecision(
                    self.state, Action.REVERSE, "Obstacle ahead; rear clearance reported"
                )
            return NavigationDecision(
                self.state, Action.STOP, "Obstacle ahead; reverse clearance unknown/low"
            )

        # A positive but not yet confirmed detection does not interrupt the
        # scan. Only a confirmed FIRE result drives alignment or approach.
        strongest = fire_result.get("strongest")
        is_fire = (
            fire_result.get("confirmed", False)
            and strongest is not None
            and strongest.get("class_name") == "FIRE"
        )
        if is_fire:
            direction = fire_result.get("fire_direction", "NONE")
            if direction in ("LEFT", "RIGHT"):
                self.state = RobotState.ALIGN_FIRE
            elif direction == "CENTER":
                self.state = RobotState.APPROACH_FIRE
                return NavigationDecision(
                    self.state, Action.FORWARD, "Fire centered; approaching"
                )
            else:
                self.state = RobotState.ALIGN_FIRE
                return NavigationDecision(
                    self.state, Action.STOP, "Fire direction uncertain"
                )
            if direction == "LEFT":
                return NavigationDecision(self.state, Action.TURN_LEFT, "Confirmed fire left")
            if direction == "RIGHT":
                return NavigationDecision(self.state, Action.TURN_RIGHT, "Confirmed fire right")

        self.state = RobotState.SCAN_FIRE
        return NavigationDecision(
            self.state, Action.SCAN_360, "No confirmed fire; scanning in place"
        )
