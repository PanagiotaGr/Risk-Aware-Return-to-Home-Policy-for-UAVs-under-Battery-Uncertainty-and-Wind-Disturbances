FROM python:3.10-slim

LABEL org.opencontainers.image.title="RiskAwareUAV-RTH"
LABEL org.opencontainers.image.description="Reproducible environment for risk-aware UAV return-to-home simulations."
LABEL org.opencontainers.image.source="https://github.com/PanagiotaGr/Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances"

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1

WORKDIR /workspace

RUN apt-get update \
    && apt-get install -y --no-install-recommends ffmpeg git \
    && rm -rf /var/lib/apt/lists/*

COPY pyproject.toml README.md ./
COPY risk_rth ./risk_rth
COPY configs ./configs
COPY scripts ./scripts
COPY tests ./tests

RUN python -m pip install --upgrade pip \
    && python -m pip install -e .[dev]

CMD ["python", "scripts/run_experiment.py", "--config", "configs/experiments/nominal.yaml", "--output-dir", "results/docker-smoke"]
