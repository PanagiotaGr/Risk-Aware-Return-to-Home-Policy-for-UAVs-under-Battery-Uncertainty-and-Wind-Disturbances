import glob
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

HOME = np.array([0.0, 0.0])
GOAL = np.array([5.0, 0.0])

HOME_EPS = 0.25
GOAL_EPS = 0.25

def classify_run(df):
    # positions
    x = df["x"].to_numpy()
    y = df["y"].to_numpy()
    b = df["battery_true"].to_numpy()

    pos = np.stack([x, y], axis=1)

    # events
    reached_goal = np.any(np.linalg.norm(pos - GOAL, axis=1) <= GOAL_EPS)
    reached_home = np.any(np.linalg.norm(pos - HOME, axis=1) <= HOME_EPS)

    crashed = np.any(b <= 0.0)

    # define metrics
    safe_return = reached_home and not crashed
    mission_success = reached_goal and safe_return
    return {
        "reached_goal": bool(reached_goal),
        "reached_home": bool(reached_home),
        "crashed": bool(crashed),
        "safe_return": bool(safe_return),
        "mission_success": bool(mission_success),
        "t_final": float(df["t"].iloc[-1]),
        "b_final": float(df["battery_true"].iloc[-1]),
    }

def main():
    log_dir = os.path.expanduser("~/uav_ws/logs")
    files = sorted(glob.glob(os.path.join(log_dir, "run_*.csv")))
    if not files:
        print("No logs found in", log_dir)
        return

    rows = []
    for f in files:
        df = pd.read_csv(f)
        r = classify_run(df)
        r["file"] = os.path.basename(f)
        rows.append(r)

    res = pd.DataFrame(rows)
    print(res[["file","mission_success","safe_return","crashed","reached_goal","reached_home","b_final"]])

    n = len(res)
    print("\nSummary over", n, "runs")
    print("Mission success rate:", res["mission_success"].mean())
    print("Safe return rate:", res["safe_return"].mean())
    print("Crash rate:", res["crashed"].mean())

    # simple bar plot
    rates = [
        res["mission_success"].mean(),
        res["safe_return"].mean(),
        res["crashed"].mean()
    ]
    labels = ["mission_success", "safe_return", "crash"]

    plt.figure()
    plt.bar(labels, rates)
    plt.ylim(0, 1)
    plt.title("Rates over runs")
    plt.show()

if __name__ == "__main__":
    main()
