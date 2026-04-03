# Risk-Aware Return-to-Home Policy for UAVs
### Python standalone implementation — no ROS 2 required

> Βασισμένο στο [PanagiotaGr/Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances](https://github.com/PanagiotaGr/Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances)

---

## Δομή project

```
uav_project/
├── uav_sim/
│   ├── state.py          # UavState dataclass  (≡ uav_interfaces/UavState.msg)
│   └── sim.py            # UAVSim              (≡ uav_sim/sim_node.py)
│
├── uav_planner/
│   ├── planner.py        # RiskAwarePlanner     (≡ uav_planner/planner_node.py)
│   └── mc_planner.py     # MCRiskAwarePlanner   (Monte Carlo extension)
│
├── uav_rl/
│   └── agent.py          # QLearningAgent       (≡ uav_reinflearning/agent_node.py)
│
├── scripts/
│   ├── run_simulation.py  # κύριο entry point   (≡ scripts/run_batch.sh)
│   └── analyze_runs.py    # metrics + plots      (≡ scripts/analyze_runs.py)
│
└── logs/                  # CSV logs + PNG plots
```

---

## Αντιστοιχία με ROS 2

| ROS 2 component | Python equivalent |
|---|---|
| `uav_interfaces/UavState.msg` | `uav_sim/state.py → UavState` |
| `uav_sim/sim_node.py` | `uav_sim/sim.py → UAVSim` |
| `uav_planner/planner_node.py` | `uav_planner/planner.py → RiskAwarePlanner` |
| `uav_reinflearning/agent_node.py` | `uav_rl/agent.py → QLearningAgent` |
| `scripts/run_batch.sh` | `scripts/run_simulation.py` |
| `scripts/analyze_runs.py` | `scripts/analyze_runs.py` |
| `/uav/cmd_vel` topic | `sim.set_cmd(vx, vy)` |
| `/uav/mode` topic | `sim.set_mode(mode)` |
| `/uav/state` topic | `sim.get_state()` |

---

## Εγκατάσταση

```bash
pip install numpy matplotlib pandas scipy
```

---

## Εκτέλεση

### Ένα run με plots
```bash
python scripts/run_simulation.py --mode single
```

### Batch evaluation (N runs)
```bash
python scripts/run_simulation.py --mode batch --n_runs 50
```

### Sensitivity sweep (z_delta / τ)
```bash
python scripts/run_simulation.py --mode sweep --n_runs 20
```

### Και τα 5 scenarios (README scenarios)
```bash
python scripts/run_simulation.py --mode scenarios
```

### Q-Learning agent training
```bash
python scripts/run_simulation.py --mode rl_train --n_runs 1000
```

---

## Αλγόριθμος RTH

### Gaussian planner (αναλυτικός)

Trigger RTH αν:
```
battery_hat  <=  K * dist_home  +  z_delta * sigma_b
```
- `K = 0.05`  (energy proxy per metre)
- `z_delta = Φ⁻¹(δ)`, π.χ. δ=0.95 → z=1.645
- `sigma_b = 0.05` (battery uncertainty std)

### Monte Carlo planner

Σε κάθε tick κάνει N samples:
```python
b_sample  ~ N(battery_hat, sigma_b)
w_sample  ~ max(0, N(|wind|, sigma_w))
cost      =  K * dist + wind_factor * max(0, headwind) * dist
safe      =  b_sample - cost > margin
```
Trigger RTH αν `P(safe) < τ`.

---

## Φυσικό μοντέλο (sim.py)

```
pos += (v_cmd + wind) * dt
drain = (α * |v_cmd| + β * |wind| + γ) * dt
battery_true -= drain
battery_hat  = battery_true + N(0, meas_sigma)
```

### Default παράμετροι
| Param | Default | Περιγραφή |
|---|---|---|
| `gust_prob` | 0.05 | πιθανότητα gust ανά tick |
| `wind_sigma` | 0.5 | std gust (m/s) |
| `meas_sigma` | 0.02 | battery measurement noise |
| `alpha` | 0.01 | drain ανά |v| |
| `beta` | 0.02 | drain ανά |wind| |
| `gamma` | 0.001 | baseline drain |
| `dt` | 0.1 | timestep (s) |

---

## Scenarios

| Scenario | gust_prob | wind_sigma | meas_sigma | Περιγραφή |
|---|---|---|---|---|
| `calm` | 0.01 | 0.1 | 0.02 | Ήρεμη πτήση |
| `headwind` | 0.02 | 0.3 | 0.02 | Αυξανόμενος αντίθετος άνεμος |
| `gusts` | 0.15 | 1.2 | 0.03 | Εκρήξεις ανέμου |
| `biased_soc` | 0.03 | 0.3 | 0.12 | Μεγάλη αβεβαιότητα μπαταρίας |
| `fault` | 0.03 | 0.4 | 0.02 | Fault injection (πολύ μεγάλο drain) |

---

## Evaluation Metrics

- **Mission success rate**: έφτασε goal AND επέστρεψε σπίτι χωρίς crash
- **Safe return rate**: επέστρεψε σπίτι χωρίς να εξαντληθεί η μπαταρία
- **Crash rate**: μπαταρία → 0 πριν επιστρέψει
- **RTH trigger time**: πότε ενεργοποιήθηκε το RTH
- **Sensitivity to τ**: sweep z_delta 0.0 → 3.0

---

## Author

**Panagiota Grosdouli** — original ROS 2 implementation  
Python standalone adaptation

## License

MIT
