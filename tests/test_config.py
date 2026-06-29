from __future__ import annotations

import json

import pytest

from risk_rth.config import ExperimentConfig, load_experiment_config


def test_default_experiment_config_is_valid() -> None:
    ExperimentConfig().validate()


def test_config_loader_reads_json(tmp_path) -> None:
    path = tmp_path / "experiment.json"
    path.write_text(
        json.dumps(
            {
                "trials": 10,
                "mc_samples": 25,
                "seed": 3,
                "output_dir": "tmp_results",
                "risk_threshold": 0.85,
            }
        ),
        encoding="utf-8",
    )

    config = load_experiment_config(path)

    assert config.trials == 10
    assert config.mc_samples == 25
    assert config.seed == 3
    assert config.output_dir == "tmp_results"
    assert config.risk_threshold == 0.85


def test_config_validation_rejects_invalid_ranges() -> None:
    with pytest.raises(ValueError):
        ExperimentConfig(trials=0).validate()
    with pytest.raises(ValueError):
        ExperimentConfig(mc_samples=0).validate()
    with pytest.raises(ValueError):
        ExperimentConfig(risk_threshold=1.5).validate()
    with pytest.raises(ValueError):
        ExperimentConfig(base_return_energy_per_m=0.0).validate()
