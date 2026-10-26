"""Finite vector validation shared by the standalone motion modules."""

import math


def vector(values, size, name, *, positive=False, nonnegative=False):
    if isinstance(values, (int, float)):
        result = (float(values),) * size
    else:
        result = tuple(float(value) for value in values)
    if len(result) != size or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain {size} finite values")
    if positive and any(value <= 0 for value in result):
        raise ValueError(f"{name} must be positive")
    if nonnegative and any(value < 0 for value in result):
        raise ValueError(f"{name} must be nonnegative")
    return result
