"""TCP server that emulates the `_1_MAIN_gcodeToMotion` EKI endpoint."""

from __future__ import annotations

import asyncio
import logging
from contextlib import suppress
from typing import Optional

from .protocol import GCodeCommand, pack_feedback
from .state import RobotState

LOGGER = logging.getLogger(__name__)


class CommandServer:
    def __init__(
        self,
        state: RobotState,
        queue: asyncio.Queue[GCodeCommand],
        *,
        host: str,
        port: int,
        feedback_interval: float = 0.05,
    ) -> None:
        self._state = state
        self._queue = queue
        self._host = host
        self._port = port
        self._feedback_interval = feedback_interval
        self._server: Optional[asyncio.base_events.Server] = None
        self._feedback_task: Optional[asyncio.Task[None]] = None
        self._writer: Optional[asyncio.StreamWriter] = None

    async def start(self) -> None:
        self._server = await asyncio.start_server(self._handle_client, self._host, self._port)
        LOGGER.info("Command server listening on %s:%s", self._host, self._port)

    async def stop(self) -> None:
        if self._feedback_task:
            self._feedback_task.cancel()
            with suppress(asyncio.CancelledError):
                await self._feedback_task
            self._feedback_task = None

        if self._server:
            self._server.close()
            await self._server.wait_closed()
            self._server = None

    async def _handle_client(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        peername = writer.get_extra_info("peername")
        LOGGER.info("Command client connected: %s", peername)
        if self._writer:
            LOGGER.info("Terminating previous command connection")
            await self._shutdown_writer()
        self._writer = writer
        self._state.set_flag(1, True)
        self._feedback_task = asyncio.create_task(self._feedback_loop())

        try:
            while True:
                payload = await reader.readexactly(48)
                command = GCodeCommand.from_bytes(payload)
                await self._queue.put(command)
                self._state.set_queue_size(self._queue.qsize())
        except asyncio.IncompleteReadError:
            LOGGER.info("Command client disconnected")
        except Exception as exc:  # noqa: BLE001
            LOGGER.exception("Command stream error: %s", exc)
        finally:
            self._state.set_flag(1, False)
            with suppress(Exception):
                await self._shutdown_writer()
            if self._feedback_task:
                self._feedback_task.cancel()
                with suppress(asyncio.CancelledError):
                    await self._feedback_task
                self._feedback_task = None

    async def _feedback_loop(self) -> None:
        if not self._writer:
            return
        writer = self._writer
        try:
            while True:
                snapshot = self._state.snapshot()
                payload = pack_feedback(snapshot.g_seq, snapshot.queue_size)
                writer.write(payload)
                await writer.drain()
                await asyncio.sleep(self._feedback_interval)
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001
            LOGGER.warning("Feedback loop stopped: %s", exc)

    async def _shutdown_writer(self) -> None:
        if not self._writer:
            return
        self._writer.close()
        with suppress(Exception):
            await self._writer.wait_closed()
        self._writer = None

