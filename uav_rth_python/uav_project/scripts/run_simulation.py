"""
scripts/run_simulation.py
--------------------------
Αντικαθιστά το run_batch.sh — τρέχει τα simulations σε Python.

Modes:
  python run_simulation.py --mode single    # ένα run με plots
  python run_simulation.py --mode batch     # N runs, batch metrics
  python run_simulation.py --mode sweep     # z_delta sweep (sensitivity)
  python run_simulation.py --mode rl_train  # εκπαίδευση Q-learning agent
  python run_simulation.py --mode scenarios # τρέχει και τα 5 scenarios
"""

import sys
import os
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

# ── project imports ───────────────────────────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uav_sim.sim    import UAVSim, SimParams
from uav_sim.state  import UavState
from uav_planner.planner    import RiskAwarePlanner, PlannerParams
from uav_planner.mc_planner import MCRiskAwarePlanner, MCPlannerParams
from uav_rl.agent           import QLearningAgent, RLAgentParams
from scripts.analyze_runs   import (
    classify_run, print_summary, plot_run,
    plot_batch_summary, plot_tau_sensitivity
)


# ─────────────────────────────────────────────────────────────────────
# Scenario definitions  (αντιστοιχούν στο README)
# ─────────────────────────────────────────────────────────────────────
SCENARIOS = {
    "calm": dict(
        gust_prob=0.01, wind_sigma=0.1,
        meas_sigma=0.02, alpha=0.01, beta=0.01, gamma=0.001,
        label="Calm flight"
    ),
    "headwind": dict(
        gust_prob=0.02, wind_sigma=0.3,
        meas_sigma=0.02, alpha=0.01, beta=0.05, gamma=0.001,
        label="Increasing headwind",
        wind_base=np.array([0.5, 0.0]),
    ),
    "gusts": dict(
        gust_prob=0.15, wind_sigma=1.2,
        meas_sigma=0.03, alpha=0.01, beta=0.03, gamma=0.001,
        label="Gust bursts"
    ),
    "biased_soc": dict(
        gust_prob=0.03, wind_sigma=0.3,
        meas_sigma=0.12,   # ← μεγάλο measurement noise
        alpha=0.01, beta=0.02, gamma=0.001,
        label="Biased SoC estimate"
    ),
    "fault": dict(
        gust_prob=0.03, wind_sigma=0.4,
        meas_sigma=0.02, alpha=0.04, beta=0.04, gamma=0.005,  # ← μεγάλο drain
        label="Fault injection (energy drain)"
    ),
}


# ─────────────────────────────────────────────────────────────────────
# Core simulation loop
# ─────────────────────────────────────────────────────────────────────
def run_episode(
    sim_params: SimParams,
    planner_type: str = "gaussian",   # "gaussian" ή "mc"
    planner_params = None,
    mc_params: MCPlannerParams = None,
    max_t: float = 60.0,
    verbose: bool = False,
    run_tag: str = "run",
) -> pd.DataFrame:
    """
    Τρέχει ένα πλήρες episode και επιστρέφει DataFrame.

    Αντιστοιχεί στον loop:
      ros2 run uav_sim uav_sim  +  ros2 run uav_planner uav_planner
    """
    sim = UAVSim(sim_params, run_tag=run_tag)

    if planner_type == "mc":
        planner = MCRiskAwarePlanner(mc_params or MCPlannerParams())
    else:
        planner = RiskAwarePlanner(planner_params or PlannerParams())

    home = sim_params.home
    goal = (planner_params or mc_params or PlannerParams()).goal

    p_safe_log = []

    while sim.t < max_t:
        state = sim.get_state()

        # ── planner decision ─────────────────────────────────────────
        if planner_type == "mc":
            vx, vy, mode, p_safe = planner.decide(state, wind_est=sim.wind)
            p_safe_log.append(p_safe)
        else:
            vx, vy, mode = planner.decide(state)

        sim.set_cmd(vx, vy)
        sim.set_mode(mode)
        state = sim.step()

        if verbose and int(sim.t / sim_params.dt) % 20 == 0:
            mode_s = "RTH" if mode == 1 else "MISSION"
            print(f"  t={sim.t:5.1f}s  pos=({state.x:5.2f},{state.y:5.2f})"
                  f"  batt={state.battery_true:.3f}  mode={mode_s}")

        # ── termination checks ────────────────────────────────────────
        dist_home = np.linalg.norm(state.pos - home)
        dist_goal = np.linalg.norm(state.pos - goal)

        if dist_goal < 0.30:
            if verbose:
                print(f"  ✓ Goal reached at t={sim.t:.1f}s!")
            break
        if mode == 1 and dist_home < 0.30:
            if verbose:
                print(f"  ✓ Returned home safely at t={sim.t:.1f}s!")
            break
        if state.battery_true <= 0.005:
            if verbose:
                print(f"  ✗ Battery depleted at t={sim.t:.1f}s!")
            break

    sim.save_csv()
    df = pd.DataFrame(sim.log)
    if p_safe_log:
        # pad to same length
        pad = len(df) - len(p_safe_log)
        p_safe_log = [1.0] * pad + p_safe_log if pad > 0 else p_safe_log[:len(df)]
        df["p_safe"] = p_safe_log
    return df


# ─────────────────────────────────────────────────────────────────────
# Modes
# ─────────────────────────────────────────────────────────────────────
def mode_single(args):
    print("\n[ Single run — MC planner ]")
    HOME = np.array([0.0, 0.0])
    GOAL = np.array([5.0, 0.0])

    sp = SimParams(
        gust_prob=0.08, wind_sigma=0.6, meas_sigma=0.03,
        alpha=0.01, beta=0.02, gamma=0.001,
        start=HOME.copy(), home=HOME.copy(), log_dir="./logs"
    )
    mp = MCPlannerParams(
        home=HOME.copy(), goal=GOAL.copy(),
        tau=0.80, n_samples=500, sigma_b=0.05
    )
    df = run_episode(sp, planner_type="mc", mc_params=mp,
                     max_t=60.0, verbose=True, run_tag="run_single")

    result = classify_run(df)
    print(f"\n  Αποτέλεσμα: {result}")

    fig = plot_run(df, title="Single Run — Risk-Aware RTH (MC Planner)",
                   save_path="./logs/single_run.png")
    plt.show()


def mode_batch(args):
    N = args.n_runs
    print(f"\n[ Batch — {N} runs, MC planner ]")
    HOME = np.array([0.0, 0.0])
    GOAL = np.array([5.0, 0.0])

    all_results = []
    for i in range(N):
        sp = SimParams(
            gust_prob=0.08, wind_sigma=0.6, meas_sigma=0.03,
            alpha=0.01, beta=0.02, gamma=0.001,
            start=HOME.copy(), home=HOME.copy(), log_dir="./logs"
        )
        mp = MCPlannerParams(
            home=HOME.copy(), goal=GOAL.copy(),
            tau=0.80, n_samples=300, sigma_b=0.05
        )
        tag = f"run_batch_{i:04d}"
        df = run_episode(sp, planner_type="mc", mc_params=mp,
                         max_t=60.0, run_tag=tag)
        r = classify_run(df)
        all_results.append(r)
        if (i + 1) % 10 == 0:
            print(f"  {i+1}/{N} runs complete...")

    results = pd.DataFrame(all_results)
    print_summary(results)
    plot_batch_summary(results, save_path="./logs/batch_summary.png")
    plt.show()


def mode_sweep(args):
    """
    Sensitivity sweep ως προς z_delta.
    Αντιστοιχεί στο Z_LIST sweep του run_batch.sh.
    """
    Z_LIST = [0.0, 0.5, 1.0, 1.645, 2.0, 2.5, 3.0]
    N = args.n_runs
    HOME = np.array([0.0, 0.0])
    GOAL = np.array([5.0, 0.0])

    print(f"\n[ Sensitivity sweep — z_delta × {N} runs each ]")

    success_r, safe_r, crash_r = [], [], []
    for z in Z_LIST:
        print(f"  z_delta={z:.3f}...")
        batch = []
        for i in range(N):
            sp = SimParams(
                gust_prob=0.08, wind_sigma=0.6, meas_sigma=0.03,
                alpha=0.01, beta=0.02, gamma=0.001,
                start=HOME.copy(), home=HOME.copy(), log_dir="./logs"
            )
            pp = PlannerParams(
                home=HOME.copy(), goal=GOAL.copy(),
                z_delta=z, sigma_b=0.05, k_energy_per_m=0.05
            )
            df = run_episode(sp, planner_type="gaussian",
                             planner_params=pp, max_t=60.0,
                             run_tag=f"sweep_z{z:.2f}_i{i:03d}")
            batch.append(classify_run(df))
        res = pd.DataFrame(batch)
        success_r.append(res["mission_success"].mean())
        safe_r.append(res["safe_return"].mean())
        crash_r.append(res["crashed"].mean())

    plot_tau_sensitivity(Z_LIST, success_r, safe_r, crash_r,
                         save_path="./logs/tau_sensitivity.png")
    plt.show()


def mode_scenarios(args):
    """Τρέχει και τα 5 scenarios, ένα plot per scenario."""
    HOME = np.array([0.0, 0.0])
    GOAL = np.array([5.0, 0.0])
    print("\n[ 5 Scenarios ]")

    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    axes = axes.flatten()

    for idx, (name, cfg) in enumerate(SCENARIOS.items()):
        label = cfg.pop("label")
        wb = cfg.pop("wind_base", np.zeros(2))
        cfg_clean = {k: v for k, v in cfg.items()}

        sp = SimParams(
            start=HOME.copy(), home=HOME.copy(), log_dir="./logs",
            wind_base=wb, **cfg_clean
        )
        mp = MCPlannerParams(
            home=HOME.copy(), goal=GOAL.copy(),
            tau=0.80, n_samples=300, sigma_b=0.05
        )
        print(f"  Running: {label}...")
        df = run_episode(sp, planner_type="mc", mc_params=mp,
                         max_t=60.0, run_tag=f"scenario_{name}")

        r = classify_run(df)
        outcome = "✓ SUCCESS" if r["mission_success"] else ("✓ SAFE RTH" if r["safe_return"] else "✗ CRASH")

        ax = axes[idx]
        mode_v = df["mode"].to_numpy()
        ax.plot(df["x"].where(mode_v==0), df["y"].where(mode_v==0), "b-", lw=1.5, label="Mission")
        ax.plot(df["x"].where(mode_v==1), df["y"].where(mode_v==1), "r-", lw=1.5, label="RTH")
        ax.plot(*HOME, "go", ms=9); ax.plot(*GOAL, "m^", ms=9)
        ax.set_title(f"{label}\n{outcome}", fontsize=9, fontweight="bold")
        ax.grid(True, alpha=0.3); ax.set_aspect("equal")
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        cfg["label"] = label
        cfg["wind_base"] = wb

    # τελευταίο panel: battery comparison
    ax = axes[5]
    for name, cfg in SCENARIOS.items():
        wb = cfg.get("wind_base", np.zeros(2))
        sp = SimParams(
            start=HOME.copy(), home=HOME.copy(), log_dir="./logs",
            wind_base=wb,
            gust_prob=cfg["gust_prob"], wind_sigma=cfg["wind_sigma"],
            meas_sigma=cfg["meas_sigma"], alpha=cfg["alpha"],
            beta=cfg["beta"], gamma=cfg["gamma"],
        )
        mp = MCPlannerParams(home=HOME.copy(), goal=GOAL.copy(), tau=0.80, n_samples=200)
        df2 = run_episode(sp, planner_type="mc", mc_params=mp, max_t=60.0, run_tag=f"batt_{name}")
        ax.plot(df2["t"], df2["battery_true"], label=cfg["label"], lw=1.2)

    ax.set_xlabel("t (s)"); ax.set_ylabel("SoC")
    ax.set_title("Battery comparison (all scenarios)")
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    plt.suptitle("5 Scenarios — Risk-Aware RTH", fontsize=13, fontweight="bold")
    plt.tight_layout()
    plt.savefig("./logs/scenarios.png", dpi=150, bbox_inches="tight")
    print("  [+] Αποθηκεύτηκε: ./logs/scenarios.png")
    plt.show()


def mode_rl_train(args):
    """Εκπαιδεύει τον Q-learning agent."""
    N_EPISODES = args.n_runs
    HOME = np.array([0.0, 0.0])
    GOAL = np.array([5.0, 0.0])

    print(f"\n[ RL Training — {N_EPISODES} episodes ]")

    agent_params = RLAgentParams(q_table_path="./logs/q_table.json")
    agent = QLearningAgent(agent_params)

    rewards_per_ep = []

    for ep in range(N_EPISODES):
        sp = SimParams(
            gust_prob=0.08, wind_sigma=0.6, meas_sigma=0.03,
            alpha=0.01, beta=0.02, gamma=0.001,
            start=HOME.copy(), home=HOME.copy(), log_dir="./logs"
        )
        sim = UAVSim(sp, run_tag=f"rl_ep_{ep:04d}")

        # navigation: απλή proportional control προς goal/home
        v_cmd_mag = 1.0
        mode = 0
        ep_reward = 0.0
        prev_state = None

        while sim.t < 60.0:
            state = sim.get_state()
            dist_home = float(np.linalg.norm(state.pos - HOME))
            wind_mag  = float(np.linalg.norm(sim.wind))

            # ── agent action ─────────────────────────────────────────
            action = agent.select_action(
                state.battery_hat, dist_home, wind_mag
            )
            if action == 1:
                mode = 1

            # ── navigation ───────────────────────────────────────────
            target = HOME if mode == 1 else GOAL
            dir_ = target - state.pos
            d = float(np.linalg.norm(dir_))
            if d > 0.05:
                dir_ = dir_ / d
                vx, vy = float(v_cmd_mag * dir_[0]), float(v_cmd_mag * dir_[1])
            else:
                vx, vy = 0.0, 0.0

            sim.set_cmd(vx, vy)
            sim.set_mode(mode)
            next_state = sim.step()

            dist_home_next = float(np.linalg.norm(next_state.pos - HOME))
            wind_mag_next  = float(np.linalg.norm(sim.wind))

            # ── reward ───────────────────────────────────────────────
            dist_goal = float(np.linalg.norm(next_state.pos - GOAL))
            done = False
            if dist_goal < 0.30:
                reward = agent_params.r_goal
                done = True
            elif mode == 1 and dist_home_next < 0.30:
                reward = agent_params.r_safe_return
                done = True
            elif next_state.battery_true <= 0.005:
                reward = agent_params.r_crash
                done = True
            else:
                reward = agent_params.r_step

            # ── Q-update ─────────────────────────────────────────────
            agent.update(
                state.battery_hat, dist_home, wind_mag, action, reward,
                next_state.battery_hat, dist_home_next, wind_mag_next, done
            )
            ep_reward += reward

            if done:
                break

        agent.end_episode()
        rewards_per_ep.append(ep_reward)

        if (ep + 1) % 100 == 0:
            print(f"  Ep {ep+1}/{N_EPISODES}  ε={agent.epsilon:.3f}  "
                  f"avg_reward={np.mean(rewards_per_ep[-100:]):.2f}")

    agent.save()
    print(f"\n  Q-table saved. Stats: {agent.q_table_stats()}")

    fig, ax = plt.subplots(figsize=(10, 4))
    window = 50
    smoothed = pd.Series(rewards_per_ep).rolling(window).mean()
    ax.plot(rewards_per_ep, alpha=0.3, color="steelblue")
    ax.plot(smoothed, color="steelblue", linewidth=2, label=f"Rolling mean ({window})")
    ax.set_xlabel("Episode")
    ax.set_ylabel("Total reward")
    ax.set_title(f"Q-Learning Training — {N_EPISODES} episodes")
    ax.legend(); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("./logs/rl_training.png", dpi=150)
    print("  [+] Αποθηκεύτηκε: ./logs/rl_training.png")
    plt.show()


# ─────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="UAV Risk-Aware RTH Simulator"
    )
    parser.add_argument(
        "--mode", choices=["single", "batch", "sweep", "scenarios", "rl_train"],
        default="single", help="Simulation mode"
    )
    parser.add_argument(
        "--n_runs", type=int, default=30,
        help="Αριθμός runs για batch/sweep/rl_train"
    )
    args = parser.parse_args()

    os.makedirs("./logs", exist_ok=True)

    if args.mode == "single":
        mode_single(args)
    elif args.mode == "batch":
        mode_batch(args)
    elif args.mode == "sweep":
        mode_sweep(args)
    elif args.mode == "scenarios":
        mode_scenarios(args)
    elif args.mode == "rl_train":
        mode_rl_train(args)


if __name__ == "__main__":
    main()
