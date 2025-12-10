"""Telemetry broadcasting, replicating `_2_telemetry.sub`."""

from __future__ import annotations

import asyncio
import logging
from contextlib import suppress
from typing import Optional, Set

from .protocol import pack_telemetry
from .state import RobotState

LOGGER = logging.getLogger(__name__)


class TelemetryBroadcaster:
    def __init__(
        self,
        state: RobotState,
        *,
        mode: str,
        host: str,
        port: int,
        interval: float = 0.02,
        retry_delay: float = 2.0,
    ) -> None:
        self._state = state
        self._mode = mode
        self._host = host
        self._port = port
        self._interval = interval
        self._retry_delay = retry_delay

        self._server: Optional[asyncio.base_events.Server] = None
        self._clients: Set[asyncio.StreamWriter] = set()
        self._broadcast_task: Optional[asyncio.Task[None]] = None
        self._client_task: Optional[asyncio.Task[None]] = None
        self._stop_event = asyncio.Event()

    async def start(self) -> None:
        if self._mode == "server":
            await self._start_server()
        elif self._mode == "client":
            self._client_task = asyncio.create_task(self._client_loop())
        else:
            raise ValueError(f"Unknown telemetry mode '{self._mode}'")
        self._broadcast_task = asyncio.create_task(self._broadcast_loop())

    async def stop(self) -> None:
        self._stop_event.set()
        if self._broadcast_task:
            self._broadcast_task.cancel()
            with suppress(asyncio.CancelledError):
                await self._broadcast_task
            self._broadcast_task = None

        if self._mode == "server":
            await self._stop_server()
        else:
            if self._client_task:
                self._client_task.cancel()
                with suppress(asyncio.CancelledError):
                    await self._client_task
                self._client_task = None

    async def _start_server(self) -> None:
        self._server = await asyncio.start_server(self._handle_client, self._host, self._port)
        LOGGER.info("Telemetry server listening on %s:%s", self._host, self._port)

    async def _stop_server(self) -> None:
        for writer in list(self._clients):
            with suppress(Exception):
                writer.close()
                await writer.wait_closed()
        self._clients.clear()
        if self._server:
            self._server.close()
            await self._server.wait_closed()
            self._server = None
        self._state.set_flag(2, False)

    async def _handle_client(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        self._clients.add(writer)
        self._state.set_flag(2, True)
        LOGGER.info("Telemetry subscriber connected: %s", writer.get_extra_info("peername"))
        try:
            await reader.read()
        finally:
            self._clients.discard(writer)
            if not self._clients:
                self._state.set_flag(2, False)
            with suppress(Exception):
                writer.close()
                await writer.wait_closed()

    async def _client_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                reader, writer = await asyncio.open_connection(self._host, self._port)
            except (ConnectionError, OSError):
                LOGGER.warning(
                    "Telemetry client failed to connect to %s:%s, retrying in %.1fs",
                    self._host,
                    self._port,
                    self._retry_delay,
                )
                await asyncio.sleep(self._retry_delay)
                continue
            LOGGER.info("Telemetry connected to %s:%s", self._host, self._port)
            self._clients = {writer}
            self._state.set_flag(2, True)
            try:
                await reader.read()
            finally:
                self._clients.clear()
                self._state.set_flag(2, False)
                with suppress(Exception):
                    writer.close()
                    await writer.wait_closed()

    async def _broadcast_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                if self._clients:
                    payload = pack_telemetry(self._state.snapshot())
                    for writer in list(self._clients):
                        try:
                            writer.write(payload)
                            await writer.drain()
                        except Exception:  # noqa: BLE001
                            LOGGER.warning("Telemetry client dropped")
                            self._clients.discard(writer)
                            with suppress(Exception):
                                writer.close()
                                await writer.wait_closed()
                    if not self._clients:
                        self._state.set_flag(2, False)
                await asyncio.sleep(self._interval)
        except asyncio.CancelledError:
            raise

