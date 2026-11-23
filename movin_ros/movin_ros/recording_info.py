"""
Command-line summary of a MOVIN SDK recording (``recording_info <file.pkl> ...``).

Prints the recording version, stream type, message count (per OSC address),
duration, the number of complete frames and, from the first one, the actor
name, bone count and detected skeleton preset.  A recording holds every OSC
message the receiver got, so besides the ``/MOVIN/Frame`` chunks it also
contains MOVIN Studio's ``/MOVIN/OSC/Status/Request`` probes (about one per
second); ``num_messages`` counts them, ``frames`` does not.  Legacy
``motion_player`` pickles and BVH files are not SDK recordings and are
reported with a friendly error (exit code 1).  Standard library plus the MOVIN
SDK core only; no ROS.
"""

from __future__ import annotations

import argparse
import collections
import sys

from movin_ros.frame_source import detect_preset, validate_recording

from movin_sdk_python import MovinFrameAssembler, OscPlayer

INFO_KEYS = ('recording', 'version', 'stream_type', 'num_messages', 'addresses', 'duration_sec',
             'created', 'frames', 'actor', 'frame_idx', 'timestamp', 'bones', 'preset')


def describe_recording(path: str) -> dict:
    """
    Return a summary dict for the SDK recording at ``path``.

    Raises ``FileNotFoundError`` for a missing file and ``ValueError`` (with
    the friendly legacy-format message from
    :func:`movin_ros.frame_source.validate_recording`) for anything that is not
    an SDK recording.
    """
    summary = validate_recording(path)
    player = OscPlayer(path)
    raw = getattr(player, '_recording', None)
    raw = raw if isinstance(raw, dict) else {}
    addresses, frames, frame = scan_messages(player)
    info = {
        'recording': path,
        'version': int(raw.get('version', 1)),
        'stream_type': summary['stream_type'],
        'num_messages': summary['num_messages'],
        'addresses': addresses,
        'duration_sec': summary['duration_sec'],
        'created': str(raw.get('created', '')),
        'frames': frames,
        'actor': '',
        'frame_idx': None,
        'timestamp': '',
        'bones': 0,
        'preset': '',
    }
    if frame is not None:
        info['actor'] = str(frame.get('actor', ''))
        info['frame_idx'] = frame.get('frame_idx')
        info['timestamp'] = str(frame.get('timestamp', ''))
        info['bones'] = len(frame.get('bones', []))
        info['preset'] = detect_preset(frame)
    return info


def scan_messages(player) -> tuple:
    """
    Read every message of a loaded ``OscPlayer`` once.

    Returns ``(addresses, frames, first_frame)``: the message count per OSC
    address (``{address: count}``, most frequent first), the number of complete
    frames the SDK assembler builds from them and the first such frame (or
    ``None``).
    """
    counts = collections.Counter()
    assembler = MovinFrameAssembler(max_ready_frames=1)
    frames = 0
    first = None
    for address, args in player.messages(realtime=False):
        counts[address] += 1
        frame = assembler.ingest(address, args, now=0.0)
        if frame is not None:
            frames += 1
            if first is None:
                first = frame
    return dict(counts.most_common()), frames, first


def format_info(info: dict) -> str:
    """Render :func:`describe_recording` output as aligned ``key: value`` lines."""
    lines = []
    for key in INFO_KEYS:
        value = info.get(key)
        if key == 'duration_sec':
            value = f'{float(value):.3f}'
        elif key == 'addresses':
            value = ', '.join(f'{address} {count}' for address, count in (value or {}).items())
            value = value or '-'
        elif value is None:
            value = '-'
        elif key in ('actor', 'timestamp', 'preset', 'created') and value == '':
            value = '-'
        lines.append(f'{key + ":":<14}{value}')
    return '\n'.join(lines)


def build_parser() -> argparse.ArgumentParser:
    """Return the argument parser of the CLI."""
    parser = argparse.ArgumentParser(
        prog='recording_info',
        description='Summarize MOVIN SDK raw-OSC recordings (.pkl written by OscRecorder, '
                    'the mocap_node record parameter, MOVIN-SDK-Python or the Isaac plugin).')
    parser.add_argument('paths', nargs='+', metavar='RECORDING',
                        help='recording file(s) to inspect')
    return parser


def main(argv=None) -> int:
    """Run the CLI; return 0 on success, 1 when any file could not be summarized."""
    args = build_parser().parse_args(argv)
    failures = 0
    for index, path in enumerate(args.paths):
        if index:
            print()
        try:
            info = describe_recording(path)
        except FileNotFoundError:
            print(f'error: {path}: no such file', file=sys.stderr)
            failures += 1
            continue
        except (OSError, ValueError) as exc:
            print(f'error: {exc}', file=sys.stderr)
            failures += 1
            continue
        print(format_info(info))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
