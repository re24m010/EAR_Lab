"""Project paths and configuration loading."""

from __future__ import annotations

import json
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "dataset.json"


def load_config(path: Path | str = DEFAULT_CONFIG) -> dict:
    with open(path, encoding="utf-8") as f:
        cfg = json.load(f)
    cfg["_config_path"] = str(Path(path).resolve())
    return cfg


def project_path(relative: str) -> Path:
    """Resolve a path from the config relative to the project root."""
    return (PROJECT_ROOT / relative).resolve()
