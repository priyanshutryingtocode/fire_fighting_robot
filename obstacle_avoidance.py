"""Small helpers for validating ultrasonic readings and motion permission."""
from dataclasses import dataclass
from typing import Optional


@dataclass
class RangeStatus:
    distance_cm: Optional[float]
    valid: bool


def validate_range(value, minimum=2.0, maximum=400.0):
    try:
        distance = float(value)
    except (TypeError, ValueError):
        return RangeStatus(None, False)
    if not minimum <= distance <= maximum:
        return RangeStatus(None, False)
    return RangeStatus(distance, True)
