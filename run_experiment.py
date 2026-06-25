from __future__ import annotations

import argparse
import csv
from pathlib import Path

from risk_rth.models import Uncertainty
from risk_rth.simulate import run_mission


def main() -> None:
    parser = argparse.ArgumentParser(description="Run risk-aware UAV return-to-home simulation")
    parser.add_argument("--threshold", type=float, default=0.70)
    parser.add_argument("--battery-wh", type=float, default=80.0)
    parser.add_argument("--wind", type=float, default=3.0, help="mean wind magnitude in m/s")
    parser.add_argument("--samples", type=int, default=800)
    parser.add_argument("--csv", type=Path, default=None)
    args = parser.parse_args()

    trace = run_mission(
        threshold=args.threshold,
        initial_soc_wh=args.battery_wh,
        uncertainty=Uncertainty(wind_mean_mps=args.wind),
        samples=args.samples,
    )
    for row in trace[:: max(1, len(trace)//12)]:
        print(f"step={row.step:03d} mode={row.mode:7s} dist={(row.x_m**2 + row.y_m**2)**0.5:7.1f}m soc={row.soc_wh:6.2f}Wh p_safe={row.probability_safe_return:.3f}")
    print(f"final_mode={trace[-1].mode} steps={len(trace)}")

    if args.csv:
        with args.csv.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["step", "x_m", "y_m", "soc_wh", "probability_safe_return", "mode"])
            writer.writeheader()
            writer.writerows(row.__dict__ for row in trace)
        print(f"wrote {args.csv}")


if __name__ == "__main__":
    main()
