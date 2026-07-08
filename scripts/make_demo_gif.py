"""Generate a code-created UAV RTH demo GIF and MP4.

The animation is generated from simulator output. It is a visualization artifact,
not experimental evidence.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt

from risk_rth.models.battery import BatteryModel
from risk_rth.models.energy import EnergyModel
from risk_rth.models.wind import WindModel
from risk_rth.planning.policies import RiskAwareMonteCarloPolicy
from risk_rth.simulation.simulator import MissionSimulator2D, SimulatorConfig


def main() -> None:
    simulator = MissionSimulator2D(
        SimulatorConfig(target_xy_m=(600.0, 120.0), max_time_s=180.0),
        BatteryModel(soc_noise_std=0.04),
        WindModel(mean_xy_mps=(-2.0, 0.5), std_mps=0.8, gust_amplitude_mps=0.8),
        EnergyModel(),
        seed=7,
    )
    result = simulator.run(RiskAwareMonteCarloPolicy(tau=0.95))
    history = result.history
    assets = Path("assets")
    videos = Path("results/videos")
    assets.mkdir(parents=True, exist_ok=True)
    videos.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(6, 5))
    ax.set_xlim(-50, 650)
    ax.set_ylim(-150, 200)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.scatter([0], [0], marker="*", s=160, label="home")
    ax.scatter([600], [120], marker="x", s=100, label="target")
    line, = ax.plot([], [], linewidth=2)
    point, = ax.plot([], [], marker="o")
    text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")
    ax.legend(loc="lower right")

    def update(frame: int):
        rows = history[: frame + 1]
        xs = [r["x_m"] for r in rows]
        ys = [r["y_m"] for r in rows]
        row = rows[-1]
        line.set_data(xs, ys)
        point.set_data([row["x_m"]], [row["y_m"]])
        p_safe = row["p_safe"]
        p_text = "n/a" if p_safe != p_safe else f"{p_safe:.2f}"
        text.set_text(
            f"t={row['time_s']:.0f}s\nSoC={row['soc']:.2f}\nP_safe={p_text}\n"
            f"tau=0.95\nRTH={bool(row['returning_home'])}\nSuccess={result.success}"
        )
        return line, point, text

    ani = animation.FuncAnimation(fig, update, frames=len(history), interval=80, blit=True)
    ani.save(assets / "demo.gif", writer="pillow", fps=12)
    try:
        ani.save(videos / "demo.mp4", writer="ffmpeg", fps=12)
    except Exception:
        # MP4 generation depends on local ffmpeg availability. The GIF remains the
        # mandatory generated artifact for environments without ffmpeg.
        pass
    plt.close(fig)


if __name__ == "__main__":
    main()
