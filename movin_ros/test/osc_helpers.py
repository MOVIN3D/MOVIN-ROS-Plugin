"""
OSC/UDP test helpers shared by the ``movin_ros`` pytest and launch_testing modules.

Turns the recorded test fixture back into real OSC packets (the SDK's test
encoder), sends them over UDP, assembles frames without the SDK receiver and
offers small polling utilities.  Kept out of ``conftest.py`` so that the tests
never ``import conftest`` (a name that clashes when several packages' test
directories are collected in one pytest session).
"""

from __future__ import annotations

import os
import socket
import struct
import time

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(TEST_DIR, 'data')
LIVE_FIXTURE = os.path.join(DATA_DIR, 'live_movinman_v3_1s.pkl')
LEGACY_FIXTURE = os.path.join(DATA_DIR, 'movinman_legacy_short.pkl')

FRAME_ADDRESS = '/MOVIN/Frame'


def osc_string(value: str) -> bytes:
    """Encode an OSC string (NUL terminated, padded to 4 bytes)."""
    encoded = value.encode('utf-8') + b'\0'
    return encoded + (b'\0' * (-len(encoded) % 4))


def osc_message(address: str, args) -> bytes:
    """Encode one OSC message with int32 / float32 / string arguments (SDK test encoder)."""
    tags = []
    payload = []
    for value in args:
        if isinstance(value, str):
            tags.append('s')
            payload.append(osc_string(value))
        elif isinstance(value, bool):
            raise TypeError('OSC bool arguments are not supported by the SDK reader')
        elif isinstance(value, int):
            tags.append('i')
            payload.append(struct.pack('>i', value))
        else:
            tags.append('f')
            payload.append(struct.pack('>f', float(value)))
    return osc_string(address) + osc_string(',' + ''.join(tags)) + b''.join(payload)


def recorded_messages(path: str) -> list:
    """Return ``[(address, args), ...]`` of every message in an SDK recording."""
    from movin_sdk_python import OscPlayer

    return list(OscPlayer(path).messages(realtime=False))


def assemble_frames(messages) -> list:
    """Assemble ``(address, args)`` messages into complete frames, oldest first."""
    from movin_sdk_python import MovinFrameAssembler

    assembler = MovinFrameAssembler(max_ready_frames=1_000_000)
    for address, args in messages:
        assembler.ingest(address, args, now=0.0)
    return list(assembler.ready_frames)


def frame_packets(messages, frame_count=None, actor=None, frame_idx_offset=0) -> list:
    """
    Group recorded messages per frame and re-encode them as OSC packets.

    Returns ``[[packet, ...], ...]``: one list of chunk packets per frame in
    recording order (``frame_count`` limits the number of frames).  ``actor``
    rewrites the actor header field and ``frame_idx_offset`` shifts
    ``frame_idx`` so several distinct streams can be synthesized from one
    recording.
    """
    frames = []
    current_key = None
    for address, args in messages:
        if address != FRAME_ADDRESS:
            continue
        args = list(args)
        key = (args[1], args[2])
        if key != current_key:
            if frame_count is not None and len(frames) >= frame_count:
                break
            frames.append([])
            current_key = key
        if actor is not None:
            args[1] = actor
        args[2] = int(args[2]) + frame_idx_offset
        frames[-1].append(osc_message(address, args))
    return frames


def send_packets(packets, address) -> int:
    """Send every packet to ``address`` over a fresh UDP socket; return the number sent."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        for packet in packets:
            sender.sendto(packet, address)
    return len(packets)


def wait_until(predicate, timeout: float = 3.0, interval: float = 0.002) -> bool:
    """Poll ``predicate`` until it is truthy or ``timeout`` seconds elapsed."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return bool(predicate())


def ready_frame_count(receiver) -> int:
    """Return the number of complete frames waiting in the receiver's queue."""
    with receiver.lock:
        return len(receiver.assembler.ready_frames)
