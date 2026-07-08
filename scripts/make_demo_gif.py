"""Generate a code-created UAV RTH demo GIF and MP4.

The animation is generated from simulator output. It is a visualization artifact,
not experimental evidence. The goal is to provide a clean project-page/README
hero animation that communicates the RTH decision mechanism without fabricating
quantitative claims.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from risk_rth.models.battery import BatteryModel
from risk_rth.models.energy import EnergyModel
from risk_rth.models.wind import WindModel
from risk_rth.planning.policies import RiskAwareMonteCarloPolicy
from risk_rth.simulation.simulator import MissionSimulator2D, SimulatorConfig

TAU = 0.95


def _safe_probability_text(value: float) -> str:
    return "not evaluated" if value != value else f"{value:.2f}"


def main() -> None:
    """Create README, website, and video demo artifacts from simulator output."""
    simulator = MissionSimulator2D(
        SimulatorConfig(target_xy_m=(620.0, 120.0), max_time_s=190.0),
        BatteryModel(soc_noise_std=0.04, reserve_soc=0.08),
        WindModel(mean_xy_mps=(-2.0, 0.5), std_mps=0.8, gust_amplitude_mps=0.8),
        EnergyModel(),
        seed=7,
    )
    result = simulator.run(RiskAwareMonteCarloPolicy(tau=TAU))
    history = result.history
    if not history:
        raise RuntimeError("Simulation produced no history; cannot generate animation.")

    assets = Path("assets")
    videos = Path("results/videos")
    website_public = Path("website/public")
    assets.mkdir(parents=True, exist_ok=True)
    videos.mkdir(parents=True, exist_ok=True)
    website_public.mkdir(parents=True, exist_ok=True)

    plt.rcParams.update(
        {
            "figure.dpi": 130,
            "savefig.dpi": 130,
            "font.size": 10,
            "axes.titlesize": 12,
            "axes.labelsize": 10,
        }
    )

    fig = plt.figure(figsize=(9.5, 5.4))
    grid = fig.add_gridspec(2, 2, width_ratios=(1.55, 1.0), height_ratios=(1.0, 1.0))
    ax_map = fig.add_subplot(grid[:, 0])
    ax_soc = fig.add_subplot(grid[0, 1])
    ax_prob = fig.add_subplot(grid[1, 1])

    ax_map.set_title("Risk-aware UAV Return-to-Home simulation")
    ax_map.set_xlim(-70, 700)
    ax_map.set_ylim(-170, 220)
    ax_map.set_xlabel("x [m]")
    ax_map.set_ylabel("y [m]")
    ax_map.grid(True, alpha=0.25)
    ax_map.scatter([0], [0], marker="*", s=190, label="home")
    ax_map.scatter([620], [120], marker="X", s=120, label="mission target")
    ax_map.text(0, 14, "home", ha="center")
    ax_map.text(620, 136, "target", ha="center")
    line, = ax_map.plot([], [], linewidth=2.3, label="trajectory")
    point, = ax_map.plot([], [], marker="o", markersize=7, label="UAV")
    rth_marker = ax_map.scatter([], [], marker="D", s=80, label="RTH trigger")
    wind_arrow = ax_map.arrow(520, -125, 0, 0, width=1.5, length_includes_head=True)
    ax_map.legend(loc="upper left", frameon=True)

    ax_soc.set_title("Battery state-of-charge")
    ax_soc.set_xlim(0, max(row["time_s"] for row in history) + 1)
    ax_soc.set_ylim(0, 1.02)
    ax_soc.set_ylabel("SoC")
    ax_soc.grid(True, alpha=0.25)
    soc_line, = ax_soc.plot([], [], linewidth=2)
    reserve_line = ax_soc.axhline(0.08, linestyle="--", linewidth=1.2, label="reserve")
    ax_soc.legend(handles=[reserve_line], loc="upper right", frameon=True)

    ax_prob.set_title("Estimated safe-return probability")
    ax_prob.set_xlim(0, max(row["time_s"] for row in history) + 1)
    ax_prob.set_ylim(0, 1.02)
    ax_prob.set_xlabel("time [s]")
    ax_prob.set_ylabel("$\\widehat{P}_{safe}$")
    ax_prob.grid(True, alpha=0.25)
    prob_line, = ax_prob.plot([], [], linewidth=2)
    tau_line = ax_prob.axhline(TAU, linestyle="--", linewidth=1.2, label=f"τ={TAU:.2f}")
    ax_prob.legend(handles=[tau_line], loc="lower left", frameon=True)

    info_box = ax_map.text(
        0.985,
        0.985,
        "",
        transform=ax_map.transAxes,
        ha="right",
        va="top",
        bbox={"boxstyle": "round,pad=0.45", "fc": "white", "ec": "0.75", "alpha": 0.9},
    )
    battery_bar_bg = Rectangle((0.03, 0.04), 0.28, 0.035, transform=ax_map.transAxes, fill=False)
    battery_bar = Rectangle((0.03, 0.04), 0.0, 0.035, transform=ax_map.transAxes)
    ax_map.add_patch(battery_bar_bg)
    ax_map.add_patch(battery_bar)
    ax_map.text(0.03, 0.082, "battery", transform=ax_map.transAxes, fontsize=9)

    rth_xy: tuple[float, float] | None = None

    def update(frame: int):
        nonlocal wind_arrow, rth_xy
        rows = history[: frame + 1]
        row = rows[-1]
        xs = [r["x_m"] for r in rows]
        ys = [r["y_m"] for r in rows]
        times = [r["time_s"] for r in rows]
        soc_values = [r["soc"] for r in rows]
        prob_values = [r["p_safe"] for r in rows]

        line.set_data(xs, ys)
        point.set_data([row["x_m"]], [row["y_m"]])
        soc_line.set_data(times, soc_values)
        prob_line.set_data(times, prob_values)
        battery_bar.set_width(0.28 * max(0.0, min(1.0, row["soc"])))

        if bool(row["returning_home"]) and rth_xy is None:
            rth_xy = (row["x_m"], row["y_m"])
        if rth_xy is not None:
            rth_marker.set_offsets([rth_xy])

        wind_arrow.remove()
        wind_arrow = ax_map.arrow(
            520,
            -125,
            18.0 * row["wind_x_mps"],
            18.0 * row["wind_y_mps"],
            width=1.5,
            length_includes_head=True,
        )
        ax_map.text(520, -145, "wind", ha="center", fontsize=9)

        status = "RETURN-TO-HOME" if bool(row["returning_home"]) else "MISSION"
        outcome = "success" if result.success else "failure/timeout"
        info_box.set_text(
            f"mode: {status}\n"
            f"t = {row['time_s']:.0f} s\n"
            f"SoC = {row['soc']:.2f}\n"
            f"P_safe = {_safe_probability_text(row['p_safe'])}\n"
            f"τ = {TAU:.2f}\n"
            f"outcome: {outcome}"
        )
        return line, point, soc_line, prob_line, battery_bar, info_box, rth_marker, wind_arrow

    ani = animation.FuncAnimation(fig, update, frames=len(history), interval=75, blit=False)
    fig.tight_layout()
    ani.save(assets / "demo.gif", writer="pillow", fps=12)
    ani.save(website_public / "demo.gif", writer="pillow", fps=12)
    try:
        ani.save(videos / "demo.mp4", writer="ffmpeg", fps=12)
    except Exception:
        # MP4 generation depends on local ffmpeg availability. The GIF remains the
        # mandatory generated artifact for environments without ffmpeg.
        pass
    plt.close(fig)


if __name__ == "__main__":
    main()
