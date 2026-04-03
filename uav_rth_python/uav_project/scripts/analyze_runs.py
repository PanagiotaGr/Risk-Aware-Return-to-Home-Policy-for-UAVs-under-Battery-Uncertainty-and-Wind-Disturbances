"""
scripts/analyze_runs.py
------------------------
Αντιστοιχεί στο scripts/analyze_runs.py του repo.

Διαβάζει όλα τα CSV logs και υπολογίζει:
  - mission success rate
  - safe return rate
  - crash rate
  - RTH trigger time distribution
  - sensitivity analysis ως προς z_delta / tau
"""

import glob
import os
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from typing import List, Dict


# ──────────────────────────────────────────────────────────────────────
# Αντιστοιχεί στη classify_run() του analyze_runs.py
# ──────────────────────────────────────────────────────────────────────
HOME = np.array([0.0, 0.0])
GOAL = np.array([5.0, 0.0])
HOME_EPS = 0.30
GOAL_EPS = 0.30


def classify_run(df: pd.DataFrame) -> Dict:
    """
    Αναλύει ένα run και επιστρέφει metrics.
    Αντιστοιχεί στη classify_run() του analyze_runs.py.
    """
    pos = df[["x", "y"]].to_numpy()
    b   = df["battery_true"].to_numpy()
    mode = df["mode"].to_numpy()

    reached_goal = bool(np.any(np.linalg.norm(pos - GOAL, axis=1) <= GOAL_EPS))
    reached_home = bool(np.any(np.linalg.norm(pos - HOME, axis=1) <= HOME_EPS))
    crashed      = bool(np.any(b <= 0.01))

    # πότε έγινε RTH trigger
    rth_idx = np.where(np.diff(mode.astype(int)) > 0)[0]
    t_vals  = df["t"].to_numpy()
    rth_trigger_t = float(t_vals[rth_idx[0]]) if len(rth_idx) > 0 else None

    return {
        "reached_goal":    reached_goal,
        "reached_home":    reached_home,
        "crashed":         crashed,
        "safe_return":     reached_home and not crashed,
        "mission_success": reached_goal and reached_home and not crashed,
        "rth_triggered":   rth_trigger_t is not None,
        "rth_trigger_t":   rth_trigger_t,
        "t_final":         float(df["t"].iloc[-1]),
        "b_final":         float(df["battery_true"].iloc[-1]),
        "b_min":           float(b.min()),
    }


def load_all_runs(log_dir: str) -> pd.DataFrame:
    """Φορτώνει όλα τα run_*.csv αρχεία."""
    files = sorted(glob.glob(os.path.join(log_dir, "run_*.csv")))
    if not files:
        print(f"[analyze] Δεν βρέθηκαν logs στο {log_dir}")
        return pd.DataFrame()

    rows = []
    for f in files:
        try:
            df = pd.read_csv(f)
            r = classify_run(df)
            r["file"] = os.path.basename(f)
            # εξαγωγή z_delta / tau από filename αν υπάρχει
            fname = os.path.basename(f)
            r["fname"] = fname
            rows.append(r)
        except Exception as e:
            print(f"  [!] Σφάλμα στο {f}: {e}")

    return pd.DataFrame(rows)


def print_summary(results: pd.DataFrame):
    n = len(results)
    if n == 0:
        print("Δεν υπάρχουν αποτελέσματα.")
        return

    print(f"\n{'='*55}")
    print(f"  Ανάλυση {n} runs")
    print(f"{'='*55}")
    print(f"  Mission success rate:  {results['mission_success'].mean()*100:.1f}%")
    print(f"  Safe return rate:      {results['safe_return'].mean()*100:.1f}%")
    print(f"  Crash rate:            {results['crashed'].mean()*100:.1f}%")
    rth_times = results[results["rth_triggered"]]["rth_trigger_t"].dropna()
    if len(rth_times):
        print(f"  Avg RTH trigger time:  {rth_times.mean():.2f}s  (σ={rth_times.std():.2f}s)")
    print(f"  Avg final battery:     {results['b_final'].mean():.4f}")
    print(f"  Avg min battery:       {results['b_min'].mean():.4f}")
    print(f"{'='*55}\n")


def plot_run(df: pd.DataFrame, title: str = "UAV Run", save_path: str = None):
    """
    Πλήρες plot ενός run:
      - trajectory (x,y)
      - battery over time
      - wind magnitude over time
      - mode over time
    """
    fig = plt.figure(figsize=(14, 9))
    fig.suptitle(title, fontsize=14, fontweight="bold")
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.4, wspace=0.35)

    # ── 1. Trajectory ──────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, 0])
    mode_vals = df["mode"].to_numpy()
    x_m = df["x"].where(mode_vals == 0).to_numpy()
    y_m = df["y"].where(mode_vals == 0).to_numpy()
    x_r = df["x"].where(mode_vals == 1).to_numpy()
    y_r = df["y"].where(mode_vals == 1).to_numpy()

    ax1.plot(x_m, y_m, "b-", linewidth=1.5, label="Mission", alpha=0.8)
    ax1.plot(x_r, y_r, "r-", linewidth=1.5, label="RTH", alpha=0.8)
    ax1.plot(*HOME, "go", markersize=10, label="Home", zorder=5)
    ax1.plot(*GOAL, "m^", markersize=10, label="Goal", zorder=5)
    # start
    ax1.plot(df["x"].iloc[0], df["y"].iloc[0], "k.", markersize=8)

    ax1.set_xlabel("x (m)")
    ax1.set_ylabel("y (m)")
    ax1.set_title("Trajectory")
    ax1.legend(fontsize=8, loc="best")
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect("equal")

    # ── 2. Battery over time ───────────────────────────────────────
    ax2 = fig.add_subplot(gs[0, 1])
    t = df["t"].to_numpy()
    ax2.plot(t, df["battery_true"], "b-", label="True SoC", linewidth=1.5)
    ax2.plot(t, df["battery_hat"],  "b--", label="Estimated SoC", alpha=0.7, linewidth=1)

    # RTH trigger line
    rth_idx = np.where(np.diff(mode_vals) > 0)[0]
    if len(rth_idx):
        ax2.axvline(t[rth_idx[0]], color="red", linestyle="--", linewidth=1.5, label="RTH trigger")

    ax2.set_xlabel("t (s)")
    ax2.set_ylabel("SoC")
    ax2.set_title("Battery State of Charge")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim(-0.05, 1.05)

    # ── 3. Wind over time ──────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 0])
    wind_mag = np.sqrt(df["wind_x"]**2 + df["wind_y"]**2)
    ax3.fill_between(t, wind_mag, alpha=0.4, color="orange")
    ax3.plot(t, wind_mag, "darkorange", linewidth=1.2)
    ax3.set_xlabel("t (s)")
    ax3.set_ylabel("|wind| (m/s)")
    ax3.set_title("Wind Magnitude")
    ax3.grid(True, alpha=0.3)

    # ── 4. Mode over time ──────────────────────────────────────────
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.step(t, mode_vals, "k-", where="post", linewidth=2)
    ax4.fill_between(t, mode_vals, step="post", alpha=0.2, color="red")
    ax4.set_yticks([0, 1])
    ax4.set_yticklabels(["MISSION", "RTH"])
    ax4.set_xlabel("t (s)")
    ax4.set_title("UAV Mode")
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"  [+] Αποθηκεύτηκε: {save_path}")
    return fig


def plot_batch_summary(results: pd.DataFrame, save_path: str = None):
    """
    Summary plots για batch runs:
      - bar chart αποτελεσμάτων
      - histogram RTH trigger times
      - battery final distribution
    """
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    fig.suptitle(f"Batch Summary — {len(results)} runs", fontsize=13, fontweight="bold")

    # ── 1. Rates bar chart ─────────────────────────────────────────
    ax = axes[0]
    rates = {
        "Mission\nSuccess": results["mission_success"].mean(),
        "Safe\nReturn":     results["safe_return"].mean(),
        "Crash":            results["crashed"].mean(),
    }
    colors = ["steelblue", "seagreen", "tomato"]
    bars = ax.bar(rates.keys(), rates.values(), color=colors, edgecolor="white", width=0.5)
    for bar, v in zip(bars, rates.values()):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.02,
                f"{v*100:.1f}%", ha="center", fontsize=10, fontweight="bold")
    ax.set_ylim(0, 1.15)
    ax.set_ylabel("Rate")
    ax.set_title("Mission Outcome Rates")
    ax.grid(axis="y", alpha=0.3)

    # ── 2. RTH trigger time histogram ─────────────────────────────
    ax = axes[1]
    rth_times = results[results["rth_triggered"]]["rth_trigger_t"].dropna()
    if len(rth_times) > 0:
        ax.hist(rth_times, bins=15, color="steelblue", edgecolor="white", alpha=0.8)
        ax.axvline(rth_times.mean(), color="red", linestyle="--", linewidth=1.5,
                   label=f"μ={rth_times.mean():.1f}s")
        ax.legend(fontsize=9)
    ax.set_xlabel("RTH trigger time (s)")
    ax.set_ylabel("Count")
    ax.set_title("RTH Trigger Time Distribution")
    ax.grid(alpha=0.3)

    # ── 3. Final battery distribution ─────────────────────────────
    ax = axes[2]
    ax.hist(results["b_final"], bins=15, color="seagreen", edgecolor="white", alpha=0.8)
    ax.axvline(results["b_final"].mean(), color="red", linestyle="--", linewidth=1.5,
               label=f"μ={results['b_final'].mean():.3f}")
    ax.legend(fontsize=9)
    ax.set_xlabel("Final battery SoC")
    ax.set_ylabel("Count")
    ax.set_title("Final Battery Distribution")
    ax.grid(alpha=0.3)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"  [+] Αποθηκεύτηκε: {save_path}")
    return fig


def plot_tau_sensitivity(
    tau_values: List[float],
    success_rates: List[float],
    safe_rates: List[float],
    crash_rates: List[float],
    save_path: str = None,
):
    """
    Sensitivity analysis: αποτελέσματα ως προς τιμές τ (risk threshold).
    Αντιστοιχεί στον Z_LIST sweep του run_batch.sh.
    """
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(tau_values, success_rates, "b-o", label="Mission Success", linewidth=2)
    ax.plot(tau_values, safe_rates,    "g-s", label="Safe Return",     linewidth=2)
    ax.plot(tau_values, crash_rates,   "r-^", label="Crash",           linewidth=2)
    ax.set_xlabel("Risk threshold τ (z_delta)")
    ax.set_ylabel("Rate")
    ax.set_title("Sensitivity Analysis: Outcome vs. Risk Threshold τ")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-0.05, 1.10)
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
    return fig


if __name__ == "__main__":
    log_dir = "./logs"
    results = load_all_runs(log_dir)
    if not results.empty:
        print_summary(results)
        plot_batch_summary(results, save_path="./logs/batch_summary.png")
        plt.show()
