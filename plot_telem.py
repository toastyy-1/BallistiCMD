#!/usr/bin/env python3

import sys
import numpy as np
import matplotlib.pyplot as plt

DEFAULT_PATH = "fc_log.csv"

GROUPS = [
    ("position",      "m",     "r"),
    ("velocity",      "m/s",   "v"),
    ("acceleration",  "m/s²",  "a"),
    ("ang. velocity", "rad/s", "w"),
]
AXES = ["x", "y", "z"]
ATTITUDE_COMPONENTS = ["w", "x", "y", "z"]

STAGE_NAMES = {
    -2: "STANDBY",
    -1: "ARMED",
     0: "STAGE_1",
     1: "STAGE_2",
     2: "PAYLOAD_DEPLOY",
}
STAGE_COLORS = {
    -2: "tab:gray",
    -1: "tab:orange",
     0: "tab:blue",
     1: "tab:green",
     2: "tab:purple",
}

def load(path):
    try:
        data = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    except OSError:
        sys.exit(f"could not open '{path}'")
    if data.size == 0:
        sys.exit(f"'{path}' has a header but no data rows")
    return np.atleast_1d(data)

# shades the background of an axis by which FC stage was active at each point in time
def shade_stages(ax, t, stage):
    change_idx = np.flatnonzero(np.diff(stage)) + 1
    starts = np.concatenate(([0], change_idx))
    ends = np.concatenate((change_idx, [len(t) - 1]))
    for start, end in zip(starts, ends):
        s = int(stage[start])
        ax.axvspan(t[start], t[end], color=STAGE_COLORS.get(s, "tab:gray"), alpha=0.08, linewidth=0)

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH
    data = load(path)

    t = data["time"]
    stage = data["stage"]

    n_rows = len(GROUPS) + 1
    n_cols = len(ATTITUDE_COMPONENTS)  # 4, wider than AXES so the attitude row fits
    fig = plt.figure(figsize=(16, 13))
    gs = fig.add_gridspec(n_rows, n_cols)
    fig.suptitle("flight controller telemetry", fontsize=14)

    for i, (label, unit, prefix) in enumerate(GROUPS):
        for j, axis in enumerate(AXES):
            ax = fig.add_subplot(gs[i, j])
            col = f"{prefix}_{axis}"
            shade_stages(ax, t, stage)
            ax.plot(t, data[col], color="tab:blue")
            ax.set_title(f"{label} {axis}", fontsize=9)
            ax.grid(True, alpha=0.3)
            if j == 0:
                ax.set_ylabel(unit)

    for k, comp in enumerate(ATTITUDE_COMPONENTS):
        ax = fig.add_subplot(gs[len(GROUPS), k])
        shade_stages(ax, t, stage)
        ax.plot(t, data[f"att_{comp}"], color="tab:red")
        ax.set_title(f"attitude {comp}", fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("time (s)")
        if k == 0:
            ax.set_ylabel("quat")

    legend_handles = [
        plt.Line2D([0], [0], color=STAGE_COLORS.get(s, "tab:gray"), lw=6, alpha=0.3, label=STAGE_NAMES.get(s, str(s)))
        for s in sorted(set(int(s) for s in stage))
    ]
    fig.legend(handles=legend_handles, loc="upper right", title="stage")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    plt.show()

if __name__ == "__main__":
    main()
