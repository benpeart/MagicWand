import serial
import argparse
import threading
import collections
import time
import sys

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

DEFAULT_PORT = "COM6"      # change for your system
BAUD_RATE = 115200
MAX_POINTS = 512           # how many recent points to show
INVALID_DATA_CLEAR_DELAY = 0.5  # seconds before clearing on invalid data

# ---------- Custom Buffer class ----------
class DataBuffer(collections.deque):
    def __init__(self, maxlen):
        super().__init__(maxlen=maxlen)
        self.invalid_data = False
        self.invalid_data_time = None

# ---------- Serial reader thread ----------
class SerialReader(threading.Thread):
    def __init__(self, port, baud, buffer):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.buffer = buffer
        self._stop = False

    def run(self):
        while not self._stop:
            try:
                with serial.Serial(self.port, self.baud, timeout=1) as ser:
                    print(f"[INFO] Connected to {self.port} @ {self.baud}")
                    while not self._stop:
                        line = ser.readline().decode("utf-8", errors="ignore").strip()
                        if not line:
                            continue
                        self._parse_and_buffer(line)
            except serial.SerialException as e:
                print(f"[WARN] Serial error: {e}")
                time.sleep(1.0)

    def _parse_and_buffer(self, line):
        """Parse comma-separated format: timestamp,x,y"""
        parts = line.split(",")
        if len(parts) < 3:
            self.buffer.invalid_data = True
            return
        try:
            ts = int(parts[0])
            x = float(parts[1])
            y = float(parts[2])
            self.buffer.invalid_data = False
            self.buffer.append((ts, x, y))
        except ValueError:
            self.buffer.invalid_data = True

    def stop(self):
        self._stop = True


# ---------- Stdin reader thread ----------
class StdinReader(threading.Thread):
    def __init__(self, buffer):
        super().__init__(daemon=True)
        self.buffer = buffer
        self._stop = False

    def run(self):
        """Read from stdin in format: timestamp,x,y"""
        print("[INFO] Reading from stdin (format: timestamp,x,y)")
        try:
            for line in sys.stdin:
                if self._stop:
                    break
                line = line.strip()
                if not line:
                    continue
                self._parse_and_buffer(line)
        except (KeyboardInterrupt, EOFError):
            pass

    def _parse_and_buffer(self, line):
        """Parse comma-separated format: timestamp,x,y"""
        parts = line.split(",")
        if len(parts) < 3:
            self.buffer.invalid_data = True
            return
        try:
            ts = int(parts[0])
            x = float(parts[1])
            y = float(parts[2])
            self.buffer.invalid_data = False
            self.buffer.append((ts, x, y))
        except ValueError:
            self.buffer.invalid_data = True

    def stop(self):
        self._stop = True


def main():
    parser = argparse.ArgumentParser(description="Real-time trajectory visualizer")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port (e.g. COM6, /dev/ttyUSB0). If not specified, reads from stdin.")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help="Baud rate (default: 115200)")
    parser.add_argument("--stdin", action="store_true", help="Read from stdin instead of serial port")
    args = parser.parse_args()

    buffer = DataBuffer(maxlen=MAX_POINTS)

    # Choose reader based on args
    if args.stdin or (args.port == DEFAULT_PORT and not args.port):
        reader = StdinReader(buffer)
    else:
        reader = SerialReader(args.port, args.baud, buffer)

    reader.start()

    plt.style.use("seaborn-v0_8")
    fig, ax = plt.subplots(figsize=(6, 6))
    line, = ax.plot([], [], "b.-", linewidth=1, markersize=3)

    ax.set_title("Real-time Wand Tip Trajectory")
    ax.set_xlabel("tip_x (world)")
    ax.set_ylabel("tip_y (world)")
    ax.set_aspect("equal", "box")
    ax.grid(True)

    def init():
        line.set_data([], [])
        return line,

    def update(frame):
        # Check if we received invalid data - clear buffer after delay
        if buffer.invalid_data:
            if buffer.invalid_data_time is None:
                buffer.invalid_data_time = time.time()
            elif time.time() - buffer.invalid_data_time > INVALID_DATA_CLEAR_DELAY:
                buffer.clear()
                line.set_data([], [])
                print("[INFO] Invalid data detected - cleared buffer, waiting for valid data...")
                return line,
        else:
            buffer.invalid_data_time = None

        if not buffer:
            return line,
        xs = [p[1] for p in buffer]
        ys = [p[2] for p in buffer]
        line.set_data(xs, ys)

        if xs and ys:
            margin = 0.05
            xmin, xmax = min(xs), max(xs)
            ymin, ymax = min(ys), max(ys)
            dx = xmax - xmin
            dy = ymax - ymin
            if dx == 0: dx = 1e-3
            if dy == 0: dy = 1e-3
            ax.set_xlim(xmin - margin * dx, xmax + margin * dx)
            ax.set_ylim(ymin - margin * dy, ymax + margin * dy)

        return line,

    ani = FuncAnimation(fig, update, init_func=init, interval=30, blit=True, cache_frame_data=False)

    try:
        plt.show()
    finally:
        reader.stop()
        print("[INFO] Exiting.")

if __name__ == "__main__":
    main()
