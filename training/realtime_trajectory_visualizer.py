import serial
import argparse
import threading
import collections
import time
import sys
import math

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

DEFAULT_PORT = "COM6"      # change for your system
BAUD_RATE = 115200
MAX_POINTS = 512           # how many recent points to show

# ---------- Custom Buffer class ----------
class DataBuffer(collections.deque):
    def __init__(self, maxlen):
        super().__init__(maxlen=maxlen)
        self.invalid_data = False
        self.on_reset = None  # callback for gesture reset
        self.just_reset = False  # flag to skip frame after reset


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

            # Detect start-of-gesture marker immediately
            if math.isnan(x) and math.isnan(y):
                self.buffer.clear()
                if self.buffer.on_reset:
                    self.buffer.on_reset()
                return

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

            # Detect start-of-gesture marker immediately
            if math.isnan(x) and math.isnan(y):
                self.buffer.clear()
                if self.buffer.on_reset:
                    self.buffer.on_reset()
                return

            self.buffer.invalid_data = False
            self.buffer.append((ts, x, y))

        except ValueError:
            self.buffer.invalid_data = True

    def stop(self):
        self._stop = True


# ---------- Main ----------
def main():
    parser = argparse.ArgumentParser(description="Real-time trajectory visualizer")
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help="Serial port (e.g. COM6, /dev/ttyUSB0). If not specified, reads from stdin.")
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

    # Reset callback: clear line, reset axes, skip next frame
    def reset_graph():
        # Clear line data
        line.set_data([], [])

        # Reset axes
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)

        # Skip two frames to avoid stale polyline
        buffer.just_reset = 2

        # Force Matplotlib to flush its internal draw cache
        fig.canvas.draw()
        fig.canvas.flush_events()

    buffer.on_reset = reset_graph

    ax.set_title("Real-time Wand Tip Trajectory")
    ax.set_xlabel("tip_x (world)")
    ax.set_ylabel("tip_z (world)")
    ax.set_aspect("equal", "box")
    ax.grid(True)

    def init():
        line.set_data([], [])
        
        # Always show unit-circle axes
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        ax.set_xticks([-1, 0, 1])
        ax.set_yticks([-1, 0, 1])
        ax.set_autoscale_on(False)
        
        return line,

    def update(frame):
        # Skip invalid lines entirely
        if buffer.invalid_data:
            buffer.invalid_data = False
            return line

        # Skip one frame after reset to avoid "closing line"
        if buffer.just_reset > 0:
            buffer.just_reset -= 1
            return line

        if not buffer:
            return line,

        xs = [p[1] for p in buffer]
        ys = [p[2] for p in buffer]
        line.set_data(xs, ys)

        return line,

    ani = FuncAnimation(fig, update, init_func=init, interval=30, blit=False, cache_frame_data=False)

    try:
        plt.show()
    finally:
        reader.stop()
        print("[INFO] Exiting.")


if __name__ == "__main__":
    main()
