"""Unit conversions used at the CARLA and logging boundaries."""

MPS_PER_MPH = 0.44704


def mph_to_mps(speed_mph: float) -> float:
    return speed_mph * MPS_PER_MPH


def mps_to_mph(speed_mps: float) -> float:
    return speed_mps / MPS_PER_MPH
