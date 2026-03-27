#!/usr/bin/env python3
"""
capture_gestures.py

Monitors a serial port for gesture streams: one header line followed by 800 sample lines.
Saves confirmed captures to data/raw/subjectXX/gesture_<label>/recNNNN.csv
"""

import argparse
import csv
import os
import re
import sys
import time
from pathlib import Path
from typing import List

import serial
import serial.tools.list_ports

SAMPLES_PER_GESTURE = 800
BASE_DIR = Path("data/raw")

def list_serial_ports() -> List[str]:
    return [p.device for p in serial.tools.list_ports.comports()]

def normalize_subject(subject_input: str) -> str:
    # If user provided a number, format as two digits subjectXX
    m = re.search(r"\d+", subject_input)
    if m:
        num = int(m.group())
        return f"subject{num:02d}"
    # otherwise sanitize and use as-is
    s = re.sub(r"[^\w\-]", "_", subject_input.strip())
    return f"subject_{s}"

def next_record_number(folder: Path) -> int:
    folder.mkdir(parents=True, exist_ok=True)
    existing = [p.name for p in folder.glob("rec*.csv")]
    maxn = 0
    for name in existing:
        m = re.match(r"rec(\d{4})\.csv$", name)
        if m:
            n = int(m.group(1))
            if n > maxn:
                maxn = n
    return maxn + 1

def parse_sample_line(line: str) -> List[str]:
    line = line.strip()
    if not line:
        return []
    if "," in line:
        return [x.strip() for x in line.split(",")]
    parts = re.split(r"\s+", line)
    return [p for p in parts if p != ""]

def save_capture(folder: Path, recnum: int, header: str, samples: List[str]) -> Path:
    filename = folder / f"rec{recnum:04d}.csv"
    with filename.open("w", newline="") as f:
        f.write(header + "\n")
        for s in samples:
            f.write(s + "\n")
    return filename

def open_serial(port: str, baud: int, timeout: float = 1.0) -> serial.Serial:
    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        return ser
    except Exception as e:
        print(f"Error opening serial port {port} at {baud} baud: {e}")
        raise

def prompt_yes_no(prompt: str) -> bool:
    while True:
        ans = input(prompt + " ").strip().lower()
        if ans in ("y", "yes"):
            return True
        if ans in ("n", "no"):
            return False
        print("Please answer 'yes' or 'no'.")

def main():
    parser = argparse.ArgumentParser(description="Capture gesture streams from serial port")
    parser.add_argument("--port", help="Serial port (e.g., COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate (default 115200)")
    args = parser.parse_args()

    # Port selection
    port = args.port
    if not port:
        ports = list_serial_ports()
        if ports:
            print("Available serial ports:")
            for p in ports:
                print("  ", p)
        port = input("Enter serial port to use: ").strip()
    baud = args.baud
    if not baud:
        baud = int(input("Enter baud rate (e.g., 115200): ").strip())

    # Subject and gesture label
    subject_input = input("Enter subject name or number: ").strip()
    subject_dirname = normalize_subject(subject_input)
    gesture_label = input("Enter gesture label (no spaces): ").strip()
    gesture_label = re.sub(r"[^\w\-]", "_", gesture_label)

    # Prepare base folder
    subject_folder = BASE_DIR / subject_dirname
    gesture_folder = subject_folder / f"gesture_{gesture_label}"
    gesture_folder.mkdir(parents=True, exist_ok=True)

    print(f"Captures will be saved under: {gesture_folder}")

    # Open serial
    try:
        ser = open_serial(port, baud, timeout=1.0)
    except Exception:
        print("Failed to open serial port. Exiting.")
        sys.exit(1)

    print("Serial port opened. Waiting for gesture streams.")
    print("Each gesture must consist of one header line followed by 800 sample lines.")
    print("Press Ctrl-C to exit.")

    try:
        while True:
            print("\nWaiting for header line...")
            # Read header: first non-empty line (wait forever)
            header = ""
            while True:
                raw = ser.readline()
                if not raw:
                    # timeout from serial read, continue waiting
                    continue
                try:
                    line = raw.decode(errors="replace").strip()
                except Exception:
                    line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    header = line
                    break

            print("Header received:")
            print("  ", header)
            print(f"Reading {SAMPLES_PER_GESTURE} sample lines...")

            samples = []
            count = 0
            sample_timeout_deadline = time.time() + 1.0
            timed_out = False
            
            while count < SAMPLES_PER_GESTURE:
                elapsed = sample_timeout_deadline - time.time()
                if elapsed <= 0:
                    print(f"  Timeout: only received {count}/{SAMPLES_PER_GESTURE} samples in 1 second. Aborting capture.")
                    timed_out = True
                    break
                
                raw = ser.readline()
                if not raw:
                    # timeout from serial read, continue waiting
                    continue
                try:
                    line = raw.decode(errors="replace").strip()
                except Exception:
                    line = raw.decode("utf-8", errors="replace").strip()
                # Accept empty sample lines as valid entries if they occur
                samples.append(line)
                count += 1
                if count % 100 == 0:
                    print(f"  {count}/{SAMPLES_PER_GESTURE} samples captured")
                # Reset timeout deadline each time we receive valid data
                sample_timeout_deadline = time.time() + 1.0

            if timed_out:
                print("Capture discarded (timeout).")
                print("Ready for next gesture.")
                continue

            print("Capture complete. Preview:")
            print("Header:", header)
            preview_n = min(5, len(samples))
            for i in range(preview_n):
                print(f"  [{i+1}] {samples[i]}")

            save = prompt_yes_no("Save this gesture? (yes/no):")
            if save:
                recnum = next_record_number(gesture_folder)
                saved_path = save_capture(gesture_folder, recnum, header, samples)
                print(f"Saved capture to: {saved_path}")
            else:
                print("Capture discarded.")

            print("Ready for next gesture.")
    except KeyboardInterrupt:
        print("\nInterrupted by user. Closing serial port and exiting.")
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
