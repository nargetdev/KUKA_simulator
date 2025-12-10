"""High level orchestrator for the KUKA network simulator."""

from __future__ import annotations

import asyncio
import contextlib
import logging
import signal
from dataclasses import dataclass
from typing import Optional

from .command_server import CommandServer
from .constants import MAXIMUM_CP_SPEED_LIMIT_M_PER_S
from .motion import MotionController
from .protocol import GCodeCommand
from .state import RobotState
from .telemetry import TelemetryBroadcaster

LOGGER = logging.getLogger(__name__)


@dataclass(slots=True)
class SimulatorConfig:
    command_host: str = "0.0.0.0"
    command_port: int = 54601
    feedback_interval: float = 0.05

    telemetry_mode: str = "server"
    telemetry_host: str = "0.0.0.0"
    telemetry_port: int = 60002
    telemetry_interval: float = 0.02

    max_speed_m_s: float = MAXIMUM_CP_SPEED_LIMIT_M_PER_S
    tick_interval: float = 0.02
    queue_size: int = 0  # 0 == infinite


class KukaSimulator:
    def __init__(self, config: SimulatorConfig) -> None:
        self._config = config
        self._state = RobotState()
        self._queue: asyncio.Queue[GCodeCommand] = asyncio.Queue(maxsize=config.queue_size or 0)
        self._command_server = CommandServer(
            self._state,
            self._queue,
            host=config.command_host,
            port=config.command_port,
            feedback_interval=config.feedback_interval,
        )
        self._telemetry = TelemetryBroadcaster(
            self._state,
            mode=config.telemetry_mode,
            host=config.telemetry_host,
            port=config.telemetry_port,
            interval=config.telemetry_interval,
        )
        self._motion = MotionController(
            self._state,
            tick_interval=config.tick_interval,
            max_speed_m_s=config.max_speed_m_s,
        )
        self._executor_task: Optional[asyncio.Task[None]] = None

    async def start(self) -> None:
        LOGGER.info("Starting KUKA simulator")
        await self._command_server.start()
        await self._telemetry.start()
        self._executor_task = asyncio.create_task(self._executor_loop())

    async def stop(self) -> None:
        LOGGER.info("Stopping KUKA simulator")
        await self._command_server.stop()
        await self._telemetry.stop()
        if self._executor_task:
            self._executor_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._executor_task
            self._executor_task = None

    async def serve(self) -> None:
        await self.start()
        loop = asyncio.get_running_loop()
        stop_future = loop.create_future()

        def _signal_handler() -> None:
            if not stop_future.done():
                stop_future.set_result(None)

        for sig in (signal.SIGINT, signal.SIGTERM):
            with contextlib.suppress(NotImplementedError):
                loop.add_signal_handler(sig, _signal_handler)

        await stop_future
        await self.stop()

    async def _executor_loop(self) -> None:
        try:
            while True:
                command = await self._queue.get()
                self._state.seq_in_flight = command.seq
                try:
                    await self._motion.execute(command)
                finally:
                    self._state.g_seq = command.seq
                    self._state.seq_in_flight = -1
                    self._state.set_queue_size(self._queue.qsize())
                    self._queue.task_done()
        except asyncio.CancelledError:
            raise


