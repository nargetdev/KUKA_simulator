# KUKA Simulator

This package emulates the network surface of the welding KUKA setup that
consumes G-code buffers via the `1__MAIN__gcodeToMotion` EKI channel and
publishes telemetry via `2__EX2__telemetryTCP`.

## Quick start

```bash
python -m kuka_simulator
```

By default the simulator binds the command server to `0.0.0.0:54601` (so a ROS2
client can push 48-byte gcode buffers) and exposes the telemetry link as a TCP
server on `0.0.0.0:60002`.  If you need to mirror the real robot more closely,
run the telemetry channel in *client* mode so the simulator connects out to the
ROS process that is listening for telemetry:

```bash
python -m kuka_simulator \
  --telemetry-mode client \
  --telemetry-target-host 192.168.0.20 \
  --telemetry-port 60002
```

The CLI exposes all timing and transport parameters.  Run `python -m
kuka_simulator --help` for the full list.

## Behavior

- Commands are decoded exactly like the KRL program: the binary buffer contains
  `seq`, `cmd_type`, the XYZABC pose, `F`, `E`, and a bitmask indicating which
  fields are active.
- Motions are replayed with a configurable tick interval (default 20 ms) and the
  feedrate is clamped to `0.1 m/s`, matching `MAXIMUM_CP_SPEED_LIMIT` from the
  reference program.
- All queue bookkeeping mirrors the logic in `_1_MAIN_gcodeToMotion.src`; the
  synchronous feedback buffer reports the last completed sequence and the
  current queue depth, with the remaining slots filled with `-27`.
- Telemetry packets reproduce the `CAST_TO` layout from `_2_telemetry.sub`
  (Cartesian pose, joint pose, queue stats, flags, and the static footer).

The internal motion model is simplified—the simulator linearly interpolates
between poses and estimates joint angles directly from the ABC orientation—but
the wire protocol matches the production robot, so existing gcode clients can
talk to it without modification.

