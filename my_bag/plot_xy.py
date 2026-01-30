import pandas as pd
import matplotlib.pyplot as plt

cols = ["sec", "nsec", "frame", "child",
        "x", "y", "z",
        "qx", "qy", "qz", "qw"]

kf = pd.read_csv("odom_kf.csv", header=None)
noisy = pd.read_csv("odom_noisy.csv", header=None)
gt = pd.read_csv("odom_gt.csv", header=None)

plt.figure(figsize=(7, 7))

plt.plot(noisy[4], noisy[5],
         label="Noisy Odometry", alpha=0.5)

plt.plot(kf[4], kf[5],
         label="Kalman Filter", linewidth=2)

plt.plot(gt[4], gt[5],
         label="Ground Truth", linestyle="--")

plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Trajectory Comparison")
plt.axis("equal")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()