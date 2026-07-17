#!/usr/bin/env python3

import sys
import numpy as np
import matplotlib.pyplot as plt

DEFAULT_PATH = "landing_errors.csv"

def load(path):
    try:
        data = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    except OSError:
        sys.exit(f"could not open '{path}'")
    if data.size == 0:
        sys.exit(f"'{path}' has a header but no data rows")
    return np.atleast_1d(data)

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH
    data = load(path)

    error = data["error_m"] / 1000  # km

    cutoff = np.percentile(error, 97)
    n_outliers = int(np.sum(error > cutoff))

    fig, ax = plt.subplots(figsize=(9, 6))
    fig.suptitle("landing error distribution", fontsize=14)

    ax.hist(error, bins=100, range=(0, cutoff), color="tab:blue")
    ax.axvline(np.mean(error), color="tab:red", linestyle="--", label=f"mean = {np.mean(error):.2f} km")
    ax.axvline(np.median(error), color="tab:orange", linestyle="--", label=f"median = {np.median(error):.2f} km")
    ax.set_xlabel("error (km)")
    ax.set_ylabel("count")
    ax.grid(True, alpha=0.3)
    if n_outliers > 0:
        ax.set_title(f"{n_outliers} outlier(s) beyond {cutoff:.1f} km not shown (max = {error.max():.1f} km)", fontsize=9)
    ax.legend()

    fig.tight_layout(rect=(0, 0, 1, 0.95))
    plt.show()

if __name__ == "__main__":
    main()
