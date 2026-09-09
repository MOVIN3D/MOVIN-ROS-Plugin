"""
Command-line summary of a MOVIN SDK recording (``recording_info <file.pkl> ...``).

Prints the recording version, stream type, message count, duration and,
from the first assembled frame, the actor name, bone count and detected
skeleton preset.  Legacy ``motion_player`` pickles and BVH files are not SDK
recordings and are reported with a friendly error (exit code 1).  Standard
library plus the MOVIN SDK core only; no ROS.
"""

from __future__ import annotations

import argparse
import sys

from movin_ros.frame_source import detect_preset, validate_recording

from movin_sdk_python import OscPlayer, peek_first_frame

INFO_KEYS = ('recording', 'version', 'stream_type', 'num_messages', 'duration_sec',
             'created', 'actor', 'frame_idx', 'timestamp', 'bones', 'preset')


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
    info = {
        'recording': path,
        'version': int(raw.get('version', 1)),
        'stream_type': summary['stream_type'],
        'num_messages': summary['num_messages'],
        'duration_sec': summary['duration_sec'],
        'created': str(raw.get('created', '')),
        'actor': '',
        'frame_idx': None,
        'timestamp': '',
        'bones': 0,
        'preset': '',
    }
    frame = peek_first_frame(path)
    if frame is not None:
        info['actor'] = str(frame.get('actor', ''))
        info['frame_idx'] = frame.get('frame_idx')
        info['timestamp'] = str(frame.get('timestamp', ''))
        info['bones'] = len(frame.get('bones', []))
        info['preset'] = detect_preset(frame)
    return info


def format_info(info: dict) -> str:
    """Render :func:`describe_recording` output as aligned ``key: value`` lines."""
    lines = []
    for key in INFO_KEYS:
        value = info.get(key)
        if key == 'duration_sec':
            value = f'{float(value):.3f}'
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
