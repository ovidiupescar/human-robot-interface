"""Face state names and mappings."""

from typing import Literal

FaceStateName = Literal["standby", "processing", "speaking", "aggressive"]

STATE_NAME_TO_INT = {
    "standby": 0,
    "processing": 1,
    "speaking": 2,
    "aggressive": 3,
}

STATE_INT_TO_NAME = {v: k for k, v in STATE_NAME_TO_INT.items()}
