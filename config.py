"""Shared paths and settings for the crop/weed pipeline."""
from pathlib import Path

ROOT = Path(__file__).parent

# Raw datasets
KAGGLE_DIR = ROOT / "kaggle-dataset" / "datasets" / "kaggle_crop_weed" / "agri_data" / "data"
ROBOFLOW_DIR = ROOT / "datasets" / "roboflow_crop_weed"   # weed-only, already split

# Prepared YOLO dataset (created by 02_prepare_dataset.py)
PREPARED_DIR = ROOT / "datasets" / "crop_weed"
DATA_YAML = PREPARED_DIR / "crop_weed.yaml"

# Outputs
RUNS_DIR = ROOT / "runs"
OUTPUTS_DIR = ROOT / "outputs"
RUN_NAME = "crop_weed"
BEST_WEIGHTS = RUNS_DIR / RUN_NAME / "weights" / "best.pt"

# Classes (same IDs as the Kaggle dataset)
CLASS_NAMES = ["crop", "weed"]

# Split and training defaults
SEED = 42
SPLIT = (0.70, 0.20, 0.10)   # train / val / test
IMG_SIZE = 320               # small = fast on CPU and on the Raspberry Pi
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
