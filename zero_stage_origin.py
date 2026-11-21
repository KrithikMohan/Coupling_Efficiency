"""Set current idle X/Y/Z controller counts to zero without motion."""

import json
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

from verify_one_step import check_error, query, read_number, send


EXPECTED = {1: -42668, 2: 0, 3: -1288123}
LOG = Path(__file__).with_name("stage_origin_log.json")


def save(record: dict) -> None:
    LOG.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    if LOG.exists():
        raise RuntimeError(f"Origin log already exists: {LOG}")
    with serial.Serial("COM4", 19200, timeout=1.0) as ser:
        time.sleep(0.2)
        before = {}
        for axis in EXPECTED:
            position = read_number(ser, f"{axis}TP?")
            status = query(ser, f"{axis}TS?")
            limits = [read_number(ser, f"{axis}SL?"), read_number(ser, f"{axis}SR?")]
            if position != EXPECTED[axis] or status not in (f"{axis}TS? @", f"{axis}TS? Q"):
                raise RuntimeError(f"Axis {axis} changed or is moving: {position}, {status}; no zero commands sent")
            before[str(axis)] = {"position": position, "status": status, "limits": limits}

        record = {"timestamp_utc": datetime.now(timezone.utc).isoformat(),
                  "port": "COM4", "before": before, "zeroed_axes": []}
        save(record)
        print(f"Recorded original counts in {LOG}", flush=True)

        for axis in EXPECTED:
            # Recheck immediately before changing each reference.
            if read_number(ser, f"{axis}TP?") != EXPECTED[axis]:
                raise RuntimeError(f"Axis {axis} changed; remaining axes left untouched")
            send(ser, f"{axis}OR")
            time.sleep(0.2)
            if read_number(ser, f"{axis}TP?") != 0:
                raise RuntimeError(f"Axis {axis} did not read back zero")
            check_error(ser, axis, "zero origin")
            record["zeroed_axes"].append(axis)
            save(record)
            print(f"Axis {axis}: {EXPECTED[axis]} -> 0", flush=True)

        record["after"] = {
            str(axis): {"position": read_number(ser, f"{axis}TP?"),
                        "limits": [read_number(ser, f"{axis}SL?"), read_number(ser, f"{axis}SR?")]}
            for axis in EXPECTED
        }
        save(record)
        if any(item["position"] != 0 for item in record["after"].values()):
            raise RuntimeError("A final axis count is not zero")
        print("All three controller counts read zero. No motion command sent.")


if __name__ == "__main__":
    main()
