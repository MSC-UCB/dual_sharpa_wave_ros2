#!/usr/bin/env python3
"""Explicit-serial inspection: no start, mode/source change or motion API calls."""

import argparse
import json
import math
from pathlib import Path
import time

from dual_sharpa_wave.mit_control import read_mit_settings
from dual_sharpa_wave.sharpa_sdk_hand import load_sdk


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', required=True)
    parser.add_argument('--discovery-timeout', type=float, default=10.0)
    parser.add_argument('--output', type=Path, help='Optional local JSON report')
    args = parser.parse_args()
    if not math.isfinite(args.discovery_timeout) or args.discovery_timeout <= 0:
        parser.error('--discovery-timeout must be finite and positive')
    sdk = load_sdk()
    manager = sdk.SharpaWaveManager.get_instance()
    deadline = time.monotonic() + args.discovery_timeout
    while args.serial not in manager.get_all_device_sn():
        if time.monotonic() >= deadline:
            raise RuntimeError(f'Discovery timed out for {args.serial}')
        time.sleep(0.1)
    try:
        hand = manager.connect(args.serial, skip_tactile=True)
        status, mode = hand.get_control_mode()
        if status.code != 0:
            raise RuntimeError(f'Mode read failed: {status.message}')
        report = json.dumps({'serial': args.serial, 'control_mode': str(mode),
                             'saved_mit_settings': read_mit_settings(hand)}, indent=2)
        if args.output:
            args.output.write_text(report + '\n')
        print(report, flush=True)
    finally:
        manager.disconnect(args.serial)


if __name__ == '__main__':
    main()
