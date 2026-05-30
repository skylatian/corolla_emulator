#!/usr/bin/env python3
"""Read lines from a serial port as they arrive (e.g. VSS speed device at 115200)."""

import argparse
import sys

import serial
from serial.tools import list_ports


DEFAULT_BAUD = 115200


def list_serial_ports() -> list[list_ports.ListPortInfo]:
    return sorted(list_ports.comports(), key=lambda p: p.device)


def choose_port(ports: list[list_ports.ListPortInfo]) -> str:
    if not ports:
        print("No serial ports found.", file=sys.stderr)
        sys.exit(1)

    print("Available serial ports:")
    for i, p in enumerate(ports, start=1):
        desc = f" — {p.description}" if p.description else ""
        print(f"  {i}. {p.device}{desc}")

    while True:
        try:
            choice = input(f"Select port [1-{len(ports)}]: ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            sys.exit(0)

        if choice.isdigit():
            idx = int(choice)
            if 1 <= idx <= len(ports):
                return ports[idx - 1].device

        print("Invalid choice, try again.")


def open_port(device: str, baud: int) -> serial.Serial:
    try:
        ser = serial.Serial(device, baud, timeout=0.1)
    except serial.SerialException as exc:
        print(f"Could not open {device}: {exc}", file=sys.stderr)
        sys.exit(1)
    return ser


def read_loop(ser: serial.Serial) -> None:
    print(f"Reading from {ser.port} at {ser.baudrate} baud (Ctrl+C to quit)\n", flush=True)
    try:
        while True:
            line = ser.readline()
            if not line:
                continue
            print(line.decode("utf-8", errors="replace").rstrip("\r\n"), flush=True)
    except KeyboardInterrupt:
        print("\nStopped.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-p",
        "--port",
        help="Serial device (e.g. /dev/cu.usbmodem101 or COM3)",
    )
    parser.add_argument(
        "-b",
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Baud rate (default: {DEFAULT_BAUD})",
    )
    parser.add_argument(
        "-l",
        "--list",
        action="store_true",
        help="List serial ports and exit",
    )
    args = parser.parse_args()

    ports = list_serial_ports()
    if args.list:
        if not ports:
            print("No serial ports found.")
        else:
            for p in ports:
                desc = f" — {p.description}" if p.description else ""
                print(f"{p.device}{desc}")
        return

    device = args.port or choose_port(ports)
    with open_port(device, args.baud) as ser:
        read_loop(ser)


if __name__ == "__main__":
    main()
