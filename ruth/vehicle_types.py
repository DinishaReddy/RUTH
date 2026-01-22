# ruth/vehicle_types.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict


@dataclass(frozen=True)
class VehicleClassParams:
    name: str
    length_m: float
    max_speed_mps: float
    accel_mps2: float
    decel_mps2: float
    reaction_time_s: float = 1.0
    pcu: float = 1.0
    standstill_gap_m: float = 2.0


DEFAULT_VEHICLE_CLASSES: Dict[str, VehicleClassParams] = {
    "car": VehicleClassParams(
        name="car",
        length_m=4.5,
        max_speed_mps=13.9,  # ~50 km/h
        accel_mps2=2.5,
        decel_mps2=4.5,
        reaction_time_s=1.0,
        pcu=1.0,
        standstill_gap_m=2.0,
    ),
    "truck": VehicleClassParams(
        name="truck",
        length_m=8.0,
        max_speed_mps=11.1,  # ~40 km/h
        accel_mps2=1.4,
        decel_mps2=3.2,
        reaction_time_s=1.2,
        pcu=1.8,
        standstill_gap_m=4.0,
    ),
}