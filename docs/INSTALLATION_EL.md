# Οδηγίες Εγκατάστασης και Εκτέλεσης

## 1. Clone repository

```bash
git clone https://github.com/PanagiotaGr/Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances.git
cd Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances
```

## 2. Virtual environment

```bash
python3 -m venv .venv
source .venv/bin/activate
```

## 3. Dependencies

```bash
pip install "numpy<2" matplotlib
```

## 4. Βασικά πειράματα

```bash
python3 experiments/run_monte_carlo_experiments.py --trials 200 --mc-samples 500
```

Το script παράγει trial-level CSV, summary CSV ανά πολιτική, summary CSV ανά scenario και plots.

## 5. Ablation studies

```bash
python3 experiments/run_ablation_studies.py --trials 200
```

## 6. Battery aging study

```bash
python3 experiments/run_battery_aging_study.py --trials 200
```

## 7. Wind gust study

```bash
python3 experiments/run_wind_gust_study.py --trials 200
```

## 8. Παραγόμενα αποτελέσματα

Τα αποτελέσματα αποθηκεύονται σε φακέλους όπως `results/`, `results_ablation/`, `results_battery_aging/` και `results_wind_gust/`.
