#!/usr/bin/env python3

import sys
import numpy as np
import matplotlib.pyplot as plt

FC_DEFAULT = "fc_telem.csv"
SIM_DEFAULT = "sim_telem.csv"

GROUPS = [
    ("position",      "m",     "r"),
    ("velocity",      "m/s",   "v"),
    ("acceleration",  "m/s²",  "a"),
    ("ang. velocity", "rad/s", "w"),
]
AXES = ["x", "y", "z"]

def load(path):
    try:
        data = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    except OSError:
        sys.exit(f"could not open '{path}'")
    if data.size == 0:
        sys.exit(f"'{path}' has a header but no data rows")
    return np.atleast_1d(data)

def main():
    fc_path = sys.argv[1] if len(sys.argv) > 1 else FC_DEFAULT
    sim_path = sys.argv[2] if len(sys.argv) > 2 else SIM_DEFAULT

    fc = load(fc_path)
    sim = load(sim_path)

    fig, axarr = plt.subplots(len(GROUPS), len(AXES), figsize=(15, 10), sharex=True)
    fig.suptitle("FC estimate (dashed) vs sim ground truth (solid)", fontsize=14)

    for i, (label, unit, prefix) in enumerate(GROUPS):
        for j, axis in enumerate(AXES):
            ax = axarr[i][j]
            col = f"{prefix}_{axis}"
            ax.plot(sim["time"], sim[col], color="tab:blue", label="sim truth")
            ax.plot(fc["time"], fc[col], color="tab:red", linestyle="--", label="fc estimate")
            ax.set_title(f"{label} {axis}", fontsize=9)
            ax.grid(True, alpha=0.3)
            if j == 0:
                ax.set_ylabel(unit)
            if i == len(GROUPS) - 1:
                ax.set_xlabel("time (s)")

    handles, labels = axarr[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper right")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    plt.show()

if __name__ == "__main__":
    main()
