"""Project paths, resolved relative to this file so nothing depends on the cwd or the machine."""
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODELS_DIR = PROJECT_ROOT / "Models"
RESULTS_DIR = PROJECT_ROOT / "M_result" / "Final"
RESULTS_DATA_DIR = RESULTS_DIR / "Data"
RESULTS_PICS_DIR = RESULTS_DIR / "Pics"
