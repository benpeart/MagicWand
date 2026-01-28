# visualize_trajectories.py
import pandas as pd
import matplotlib.pyplot as plt

# CSV format: timestamp_us, tip_x, tip_y
df = pd.read_csv("trajectories.csv", header=None,
                 names=["ts", "tip_x", "tip_y"])

# Optional: segment by gesture ID if present
# e.g., ts, tip_x, tip_y, label

plt.figure(figsize=(6, 6))
plt.plot(df["tip_x"], df["tip_y"], marker='.', linewidth=1)
plt.gca().set_aspect('equal', 'box')
plt.title("Wand Tip Trajectory")
plt.xlabel("X (world)")
plt.ylabel("Y (world)")
plt.grid(True)
plt.show()
