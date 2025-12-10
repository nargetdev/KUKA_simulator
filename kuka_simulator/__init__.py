"""
KUKA simulator package.

The public API intentionally remains small; prefer to launch the simulator via
`python -m kuka_simulator` or by importing `KukaSimulator` from
`kuka_simulator.simulator`.
"""

from .simulator import KukaSimulator, SimulatorConfig

__all__ = ["KukaSimulator", "SimulatorConfig"]

