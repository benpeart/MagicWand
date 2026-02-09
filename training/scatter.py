import serial
import argparse
import threading
import queue
import matplotlib.pyplot as plt
import numpy as np


# ---------------------------------------------------------
# Serial reader thread
# ---------------------------------------------------------
def serial_reader(port, baud, q):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"[INFO] Connected to {port} at {baud} baud")
    except Exception as e:
        print(f"[ERROR] Could not open serial port: {e}")
        return

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            parts = line.split(",")
            if len(parts) != 3:
                continue

            ts, xs, ys = parts

            # Reset packet
            if ts == "0" and xs.lower() == "nan" and ys.lower() == "nan":
                q.put(("reset", None))
                continue

            # Parse numeric values
            try:
                x = float(xs)
                y = float(ys)
            except ValueError:
                continue

            q.put(("point", (x, y)))

        except Exception as e:
            print(f"[ERROR] Serial read error: {e}")
            break


# ---------------------------------------------------------
# Main plotting logic
# ---------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Real-time scatter plot from serial CSV stream"
    )
    parser.add_argument(
        "--port", type=str, default="COM6", help="COM port (e.g., COM6 or /dev/ttyUSB0)"
    )
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate")
    args = parser.parse_args()

    q = queue.Queue()

    # Start serial thread
    t = threading.Thread(
        target=serial_reader, args=(args.port, args.baud, q), daemon=True
    )
    t.start()

    # Matplotlib setup
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_autoscale_on(False)
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Real-Time Scatter Plot")

    xs = []
    ys = []

    scatter = ax.scatter([], [])

    while True:
        try:
            # Exit if window is closed
            if not plt.fignum_exists(fig.number):
                print("[INFO] Plot window closed — exiting.")
                break

            # Process all queued items
            while not q.empty():
                msg_type, payload = q.get()

                if msg_type == "reset":
                    xs.clear()
                    ys.clear()
                    scatter.set_offsets(np.empty((0, 2)))
                    fig.canvas.draw()
                    fig.canvas.flush_events()
                    print("[INFO] Reset received — clearing plot")
                    continue

                if msg_type == "point":
                    x, y = payload
                    xs.append(x)
                    ys.append(y)

            # Update scatter plot
            if xs:
                pts = np.column_stack((xs, ys))
                scatter.set_offsets(pts)

            fig.canvas.draw()
            fig.canvas.flush_events()

        except KeyboardInterrupt:
            print("\n[INFO] Exiting.")
            break


if __name__ == "__main__":
    main()
