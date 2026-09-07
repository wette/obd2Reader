#!/usr/bin/env python3
import argparse
import sys
import time

import serial
from dtc_descriptions import DTC_DESCRIPTIONS


def describe_dtc(code: str) -> str:
    code = code.strip().upper()
    if code in DTC_DESCRIPTIONS:
        return DTC_DESCRIPTIONS[code]

    family = code[:1]
    category = {
        "P": "Powertrain",
        "C": "Chassis",
        "B": "Body",
        "U": "Network",
    }.get(family, "Unknown")
    return f"{category} DTC (description not in local database)"


class ObdSerialClient:
    def __init__(self, port: str, baud: int, timeout: float = 0.2) -> None:
        self.serial = serial.Serial(port=port, baudrate=baud, timeout=timeout)

    def close(self) -> None:
        self.serial.close()

    def send_command(self, command: str) -> None:
        self.serial.write((command.strip().upper() + "\n").encode("ascii"))
        self.serial.flush()

    def _read_line(self, max_wait_s: float = 5.0) -> str:
        start = time.time()
        while time.time() - start < max_wait_s:
            raw = self.serial.readline()
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                continue
            if line:
                return line
        raise TimeoutError("Timed out waiting for Arduino response")

    def wait_until_ready(self, max_wait_s: float = 10.0) -> None:
        start = time.time()
        while time.time() - start < max_wait_s:
            line = self._read_line(max_wait_s=1.0)
            if line == "READY":
                return
            self._handle_unsolicited_line(line)
        raise TimeoutError("Did not receive READY from Arduino")

    def set_offline_mode(self, enabled: bool) -> None:
        expected = "OFFLINE ON" if enabled else "OFFLINE OFF"
        self.send_command(expected)

        while True:
            line = self._read_line(max_wait_s=5.0)
            if line == expected:
                return
            if line == f"ERR Unknown command: {expected}":
                raise RuntimeError(line)
            if line.startswith("ERR "):
                continue
            self._handle_unsolicited_line(line)

    def read_dtcs(self) -> None:
        self.send_command("READ")

        count_from_arduino = None
        seen_codes = []
        while True:
            line = self._read_line(max_wait_s=5.0)
            if line.startswith("DTC "):
                code = line[4:].strip().upper()
                seen_codes.append(code)
                print(f"{code}: {describe_dtc(code)}")
            elif line.startswith("END "):
                try:
                    count_from_arduino = int(line.split()[1])
                except (ValueError, IndexError):
                    pass
                break
            elif line.startswith("ERR"):
                raise RuntimeError(line)
            else:
                self._handle_unsolicited_line(line)

        if not seen_codes:
            print("No active OBD-II DTCs reported.")
        if count_from_arduino is not None:
            print(f"Reported DTC count: {count_from_arduino}")

    def clear_dtcs(self) -> None:
        self.send_command("CLEAR")

        while True:
            line = self._read_line(max_wait_s=5.0)
            if line == "CLEAR OK":
                print("ECU confirmed DTC clear request.")
                return
            if line.startswith("CLEAR FAIL"):
                raise RuntimeError(line)
            if line.startswith("ERR"):
                raise RuntimeError(line)
            self._handle_unsolicited_line(line)

    @staticmethod
    def _handle_unsolicited_line(line: str) -> None:
        if line.startswith("INFO"):
            return
        if line.startswith("DTC "):
            code = line[4:].strip().upper()
            print(f"{code}: {describe_dtc(code)}")
            return
        if line.startswith("END "):
            return
        print(f"[arduino] {line}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="BMW E87 OBD-II DTC serial client")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyUSB0 or COM3")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--offline", action="store_true", help="Enable Arduino offline simulation mode")
    parser.add_argument("--clear", action="store_true", help="Clear DTCs after reading")
    parser.add_argument("--no-read", action="store_true", help="Skip reading DTCs")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        client = ObdSerialClient(port=args.port, baud=args.baud)
    except serial.SerialException as exc:
        print(f"Failed to open serial port: {exc}", file=sys.stderr)
        return 2

    try:
        client.wait_until_ready()

        if args.offline:
            client.set_offline_mode(True)

        if not args.no_read:
            client.read_dtcs()

        if args.clear:
            answer = input("Clear DTCs now? Type 'yes' to continue: ").strip().lower()
            if answer == "yes":
                client.clear_dtcs()
            else:
                print("Clear operation canceled.")

        return 0
    except (TimeoutError, RuntimeError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
