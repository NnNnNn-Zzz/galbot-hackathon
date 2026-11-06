#!/usr/bin/env python3
"""Stream a raw 16 kHz, 16-bit little-endian mono PCM file through GalbotSDK."""
from __future__ import annotations
import argparse
import time
from pathlib import Path
from galbot_sdk.g1 import GalbotRobot

CHUNK_BYTES = 2560  # 80 ms at 16 kHz, 16-bit mono
BYTES_PER_SECOND = 32000


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcm", help="Path to raw mono PCM (16 kHz, signed 16-bit little-endian)")
    parser.add_argument("--volume", type=float, default=60.0)
    args = parser.parse_args()
    path = Path(args.pcm)
    payload = path.read_bytes()
    if not payload or len(payload) % 2:
        raise SystemExit("PCM must be non-empty and contain whole 16-bit samples")

    robot = GalbotRobot()
    if not robot.init():
        raise SystemExit("GalbotRobot.init() failed")
    try:
        robot.set_volume(args.volume)
        stream_id = f"pcm_{int(time.time())}"
        for offset in range(0, len(payload), CHUNK_BYTES):
            if not robot.write_audio_stream_output(payload[offset:offset + CHUNK_BYTES], stream_id):
                raise RuntimeError(f"audio write failed at byte {offset}")
            time.sleep(0.05)
        time.sleep(len(payload) / BYTES_PER_SECOND + 0.25)
        print(f"Played {path.name}: {len(payload)} bytes, {len(payload) / BYTES_PER_SECOND:.2f}s")
    finally:
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
