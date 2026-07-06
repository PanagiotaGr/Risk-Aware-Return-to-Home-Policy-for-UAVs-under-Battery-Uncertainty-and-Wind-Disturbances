from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot platform benchmark per-trial outputs.")
    parser.add_argument("--trials", default="results_platform/platform_trials.csv")
    parser.add_argument("--output-dir", default="results_platform/figures")
    args = parser.parse_args()

    data = pd.read_csv(args.trials)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.plot(data["trial_id"], data["safe_return_probability"], marker="o", linewidth=1)
    ax.set_title("Safe-return probability per trial")
    ax.set_xlabel("Trial")
    ax.set_ylabel("Safe-return probability")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "safe_return_probability.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.plot(data["trial_id"], data["energy_margin"], marker="o", linewidth=1)
    ax.set_title("Energy margin per trial")
    ax.set_xlabel("Trial")
    ax.set_ylabel("Energy margin")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "energy_margin.png", dpi=180)
    plt.close(fig)

    counts = data["decision"].value_counts().sort_index()
    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.bar(counts.index, counts.values)
    ax.set_title("Decision distribution")
    ax.set_xlabel("Decision")
    ax.set_ylabel("Count")
    ax.tick_params(axis="x", rotation=30)
    fig.tight_layout()
    fig.savefig(output_dir / "decision_distribution.png", dpi=180)
    plt.close(fig)

    print(f"Saved figures to {output_dir}")


if __name__ == "__main__":
    main()
