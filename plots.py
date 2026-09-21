import matplotlib.pyplot as plt
import numpy as np
from pendulum import load_pickle
from constants import *

def plot_all_u_trajectories(path=TVLQR_FILE):
    gains = load_pickle(path)

    plt.figure(figsize=(12, 7))

    for transition, (_, _, u_traj) in gains.items():
        times = np.linspace(
            u_traj.start_time(),
            u_traj.end_time(),
            500
        )

        forces = np.array([
            u_traj.value(t).item()
            for t in times
        ])

        plt.plot(times, forces, label=transition)

        print(
            f"{transition}: "
            f"max = {np.max(forces):.2f} N, "
            f"min = {np.min(forces):.2f} N, "
            f"max abs = {np.max(np.abs(forces)):.2f} N"
        )

    plt.axhline(0, linewidth=0.8)
    plt.axhline(50, linestyle="--", linewidth=0.8)
    plt.axhline(-50, linestyle="--", linewidth=0.8)

    plt.xlabel("Time (s)")
    plt.ylabel("Actuation force (N)")
    plt.title("TVLQR Actuation Trajectories")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_all_u_trajectories()
