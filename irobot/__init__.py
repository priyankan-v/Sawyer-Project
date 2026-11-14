from __future__ import annotations

from importlib import import_module

from irobot.src.robots.sawyer import SawyerBase, SawyerConfig, SawyerController

__all__ = [
    'CrazyflieBase',
    'CrazyflieConfig',
    'CrazyflieController',
    'SawyerBase',
    'SawyerConfig',
    'SawyerController',
]


def __getattr__(name):
    """Load the Crazyflie SDK only when that robot is requested."""
    if name in ('CrazyflieBase', 'CrazyflieConfig', 'CrazyflieController'):
        module = import_module('irobot.src.robots.crazyflie')
        return getattr(module, name)
    raise AttributeError(name)
