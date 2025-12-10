import struct
import unittest

from kuka_simulator.protocol import GCodeCommand, pack_feedback, pack_telemetry
from kuka_simulator.state import RobotState


class ProtocolTests(unittest.TestCase):
    def test_gcode_round_trip(self) -> None:
        command = GCodeCommand(
            seq=42,
            cmd_type=0,
            x=100.0,
            y=200.0,
            z=300.0,
            a=0.0,
            b=0.0,
            c=90.0,
            f=50.0,
            e=1.0,
            bitmask=0x3F,
        )
        payload = command.to_bytes()
        self.assertEqual(len(payload), 48)
        decoded = GCodeCommand.from_bytes(payload)
        self.assertEqual(decoded.seq, command.seq)
        self.assertAlmostEqual(decoded.x, command.x)
        self.assertEqual(decoded.bitmask, command.bitmask)

    def test_feedback_packet_layout(self) -> None:
        payload = pack_feedback(seq=5, queue_size=7)
        self.assertEqual(len(payload), 48)
        ints = struct.unpack("<12i", payload)
        self.assertEqual(ints[0], 5)
        self.assertEqual(ints[1], 7)
        self.assertTrue(all(value == -27 for value in ints[2:]))

    def test_telemetry_packet_size(self) -> None:
        state = RobotState()
        payload = pack_telemetry(state.snapshot())
        self.assertEqual(len(payload), 128)


if __name__ == "__main__":
    unittest.main()

