"""Command line entry point."""

from __future__ import annotations

import argparse
import asyncio
import logging
from typing import Sequence

from .simulator import KukaSimulator, SimulatorConfig


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="KUKA network simulator")
    parser.add_argument("--command-host", default="0.0.0.0")
    parser.add_argument("--command-port", type=int, default=54601)
    parser.add_argument("--feedback-interval", type=float, default=0.05)

    parser.add_argument("--telemetry-mode", choices=("server", "client"), default="server")
    parser.add_argument("--telemetry-host", default="0.0.0.0")
    parser.add_argument("--telemetry-port", type=int, default=60002)
    parser.add_argument("--telemetry-interval", type=float, default=0.02)

    parser.add_argument("--max-speed", type=float, default=0.1, help="Maximum linear speed (m/s)")
    parser.add_argument("--tick-interval", type=float, default=0.02, help="Simulation tick interval (s)")
    parser.add_argument("--queue-size", type=int, default=0, help="Incoming command queue size (0 = unlimited)")

    parser.add_argument(
        "--telemetry-target-host",
        default="127.0.0.1",
        help="When telemetry runs in client mode, connect to this host instead of binding.",
    )

    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
        help="Logging verbosity",
    )
    return parser.parse_args(argv)


async def run_from_cli(argv: Sequence[str] | None = None) -> None:
    args = parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    telemetry_host = args.telemetry_host
    telemetry_mode = args.telemetry_mode
    if telemetry_mode == "client":
        telemetry_host = args.telemetry_target_host

    config = SimulatorConfig(
        command_host=args.command_host,
        command_port=args.command_port,
        feedback_interval=args.feedback_interval,
        telemetry_mode=telemetry_mode,
        telemetry_host=telemetry_host,
        telemetry_port=args.telemetry_port,
        telemetry_interval=args.telemetry_interval,
        max_speed_m_s=args.max_speed,
        tick_interval=args.tick_interval,
        queue_size=args.queue_size,
    )

    simulator = KukaSimulator(config)
    await simulator.serve()

