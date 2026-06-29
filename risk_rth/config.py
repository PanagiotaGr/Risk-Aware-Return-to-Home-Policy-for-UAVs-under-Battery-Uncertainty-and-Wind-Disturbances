"""Configuration models and loaders for reproducible UAV RTH experiments."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class ExperimentConfig:
    """Common simulation parameters shared by standalone experiments."""

    trials: int = 200
    mc_samples: int = 500
    seed: int = 7
    output_dir: str = "results"

    deterministic_soc_threshold: float = 0.30
    risk_threshold: float = 0.90
    adaptive_base_threshold: float = 0.88

    base_return_energy_per_m: float = 0.00042
    safety_reserve_soc: float = 0.08
    early_rth_margin: float = 0.18

    def validate(self) -> None:
        """Validate numeric ranges that affect safety-critical simulations."""

        if self.trials <= 0:
            raise ValueError("trials must be positive")
        if self.mc_samples <= 0:
            raise ValueError("mc_samples must be positive")
        for name in (
            "deterministic_soc_threshold",
            "risk_threshold",
            "adaptive_base_threshold",
            "safety_reserve_soc",
            "early_rth_margin",
        ):
            value = getattr(self, name)
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        if self.base_return_energy_per_m <= 0.0:
            raise ValueError("base_return_energy_per_m must be positive")


def load_experiment_config(path: str | Path) -> ExperimentConfig:
    """Load an :class:`ExperimentConfig` from a JSON file."""

    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as handle:
        payload: dict[str, Any] = json.load(handle)

    config = ExperimentConfig(**payload)
    config.validate()
    return config
