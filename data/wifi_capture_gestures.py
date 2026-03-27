#!/usr/bin/env python3
"""
wifi_capture_gestures.py

Listens on a TCP socket for gesture streams from the WiFi capture task:
one header line followed by sample lines. Saves confirmed captures to 
data/raw/subjectXX/gesture_<label>/recNNNN.csv
"""

import argparse
import csv
import os
import re
import socket
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

BASE_DIR = Path("data/raw")
SOCKET_TIMEOUT = 5.0  # Timeout for reading individual lines
SAMPLES_PER_GESTURE = 801  # Must match INFERENCE_WINDOW_SAMPLES from data_types.h


def normalize_subject(subject_input: str) -> str:
    """Format subject name consistently."""
    # If user provided a number, format as two digits subjectXX
    m = re.search(r"\d+", subject_input)
    if m:
        num = int(m.group())
        return f"subject{num:02d}"
    # otherwise sanitize and use as-is
    s = re.sub(r"[^\w\-]", "_", subject_input.strip())
    return f"subject_{s}"


def next_record_number(folder: Path) -> int:
    """Find the next available record number."""
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


def save_capture(folder: Path, recnum: int, header: str, samples: List[str]) -> Path:
    """Save gesture data to CSV file."""
    filename = folder / f"rec{recnum:04d}.csv"
    with filename.open("w", newline="") as f:
        f.write(header + "\n")
        for s in samples:
            f.write(s + "\n")
    return filename


def prompt_yes_no(prompt: str) -> bool:
    """Ask user for yes/no confirmation."""
    while True:
        ans = input(prompt + " ").strip().lower()
        if ans in ("y", "yes"):
            return True
        if ans in ("n", "no"):
            return False
        print("Please answer 'yes' or 'no'.")


def read_line_from_socket(sock: socket.socket) -> Optional[str]:
    """
    Read a single line from socket, handling timeout.
    Returns None on timeout or socket error.
    """
    try:
        data = b""
        while b"\n" not in data:
            chunk = sock.recv(1)
            if not chunk:
                return None  # Connection closed
            data += chunk
        return data.decode(errors="replace").strip()
    except socket.timeout:
        return None
    except Exception as e:
        print(f"Socket error: {e}")
        return None


def receive_gesture(sock: socket.socket) -> Tuple[Optional[str], List[str]]:
    """
    Receive a complete gesture from socket.
    Returns (header, samples) or (None, []) on error.
    Expects exactly SAMPLES_PER_GESTURE samples after the header.
    No timeout between samples - connection/line timeout only.
    """
    import time as time_module
    
    print("\nWaiting for header line...")
    
    # Read header
    header = None
    header_time = time_module.time()
    while header is None:
        line = read_line_from_socket(sock)
        if line is None:
            print("ERROR: Socket timeout waiting for header.")
            return None, []
        if line:  # Non-empty line
            header = line
            break
    
    print("Header received:")
    print("  ", header)
    print(f"Reading {SAMPLES_PER_GESTURE} sample lines...")
    
    samples = []
    sample_start_time = time_module.time()
    first_sample_time = None
    last_sample_time = sample_start_time
    
    while len(samples) < SAMPLES_PER_GESTURE:
        line = read_line_from_socket(sock)
        
        if line is None:
            # Connection timeout or closed
            now = time_module.time()
            if len(samples) == 0:
                elapsed_since_header = now - header_time
                print(f"\nDEBUG: Socket closed immediately after header")
                print(f"       Time since header: {elapsed_since_header:.2f}s")
                print(f"       Samples received: {len(samples)}/{SAMPLES_PER_GESTURE}")
                print(f"       This suggests ESP32 closed connection before sending samples")
            else:
                elapsed = now - sample_start_time
                elapsed_since_last = now - last_sample_time
                avg_sample_rate = len(samples) / elapsed if elapsed > 0 else 0
                print(f"\nDEBUG: Socket closed mid-gesture")
                print(f"       Samples received: {len(samples)}/{SAMPLES_PER_GESTURE} ({100*len(samples)/SAMPLES_PER_GESTURE:.1f}%)")
                print(f"       Time since first sample: {elapsed:.2f}s")
                print(f"       Time since last sample: {elapsed_since_last:.2f}s")
                print(f"       Average sample rate: {avg_sample_rate:.1f} samples/sec")
                print(f"       Estimated completion time: {SAMPLES_PER_GESTURE/avg_sample_rate:.2f}s")
                print(f"       This suggests network issue or ESP32 crash mid-gesture")
            break
        
        # We got a line - add it
        now = time_module.time()
        if first_sample_time is None:
            first_sample_time = now
            time_to_first = now - header_time
            print(f"  First sample received after {time_to_first:.2f}s")
        
        samples.append(line)
        last_sample_time = now
        
        if len(samples) % 100 == 0:
            print(f"  {len(samples)}/{SAMPLES_PER_GESTURE} samples captured")
    
    # Check if we got the expected number of samples
    if len(samples) == SAMPLES_PER_GESTURE:
        print(f"\n✓ VALID gesture: {SAMPLES_PER_GESTURE} samples received")
        return header, samples
    else:
        print(f"\n✗ INVALID gesture: received {len(samples)}/{SAMPLES_PER_GESTURE} samples")
        print(f"   Discarding incomplete gesture without saving")
        return None, []


def main():
    parser = argparse.ArgumentParser(
        description="Receive gesture data from WiFi capture task"
    )
    parser.add_argument("--host", default="0.0.0.0", help="Listen host (default 0.0.0.0)")
    parser.add_argument("--port", type=int, default=5000, help="Listen port (default 5000)")
    args = parser.parse_args()

    host = args.host
    port = args.port

    # Get subject and gesture label from user
    subject_input = input("Enter subject name or number: ").strip()
    subject_dirname = normalize_subject(subject_input)
    gesture_label = input("Enter gesture label (no spaces): ").strip()
    gesture_label = re.sub(r"[^\w\-]", "_", gesture_label)

    # Prepare base folder
    subject_folder = BASE_DIR / subject_dirname
    gesture_folder = subject_folder / f"gesture_{gesture_label}"
    gesture_folder.mkdir(parents=True, exist_ok=True)

    print(f"\nCaptures will be saved under: {gesture_folder}")
    print(f"Listening on {host}:{port}")
    print("Press Ctrl-C to exit.\n")

    # Create and bind socket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, port))
        sock.listen(1)
        print(f"Server listening on {host}:{port}")
    except Exception as e:
        print(f"Failed to create socket: {e}")
        sys.exit(1)

    try:
        while True:
            print("\nWaiting for ESP32 connection...")
            try:
                client_sock, client_addr = sock.accept()
                print(f"Connected from {client_addr}")
                client_sock.settimeout(SOCKET_TIMEOUT)

                # Receive gesture
                header, samples = receive_gesture(client_sock)
                client_sock.close()

                if header is None or len(samples) != SAMPLES_PER_GESTURE:
                    # Invalid gesture - skip without prompting
                    print("Skipping incomplete gesture.")
                    print("Ready for next connection.\n")
                    continue

                print(f"Capture complete. Total samples: {len(samples)}")
                print("Preview:")
                print("Header:", header)
                preview_n = min(5, len(samples))
                for i in range(preview_n):
                    print(f"  [{i+1}] {samples[i]}")
                if len(samples) > preview_n:
                    print(f"  ... ({len(samples) - preview_n} more samples)")

                save = prompt_yes_no("Save this gesture? (yes/no):")
                if save:
                    recnum = next_record_number(gesture_folder)
                    saved_path = save_capture(gesture_folder, recnum, header, samples)
                    print(f"Saved capture to: {saved_path}")
                else:
                    print("Capture discarded.")

                print("Ready for next connection.\n")

            except KeyboardInterrupt:
                raise
            except Exception as e:
                print(f"Error: {e}")
                print("Ready for next connection.\n")
    finally:
        try:
            sock.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
