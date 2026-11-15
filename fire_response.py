"""Fire response workflow placeholder; actuation is deliberately not enabled."""
from enum import Enum, auto


class FireResponseState(Enum):
    SEARCH = auto()
    CONFIRM = auto()
    ALIGN = auto()
    APPROACH = auto()
    AIM = auto()
    SUPPRESS = auto()
    VERIFY = auto()


class FireResponse:
    """
    Workflow tracker only. It does not activate a pump or move the arm.
    Add those actions only after hardware, extinguishing medium, and safety
    interlocks have been tested independently.
    """
    def __init__(self):
        self.state = FireResponseState.SEARCH
