"""Download the Roboflow crop/weed dataset (YOLOv8 format) into ./datasets/roboflow_crop_weed.

Setup:  pip install roboflow
Key:    put ROBOFLOW_API_KEY=your_key in ML-model/.env (gitignored), or set it as an
        environment variable. Never hard-code it.
Run:    python download_roboflow.py
"""
import os
import sys
from pathlib import Path

from roboflow import Roboflow

WORKSPACE = "manvitha-dungi"
PROJECT = "weed-crop-j7jyw-itvsq"
VERSION = 1
DEST = Path(__file__).parent / "datasets" / "roboflow_crop_weed"

def load_env_file(path):
    """Minimal .env reader: KEY=value lines; real env vars take priority."""
    if not path.is_file():
        return
    for line in path.read_text(encoding="utf-8-sig").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip().strip("\"'"))


load_env_file(Path(__file__).parent / ".env")

api_key = os.environ.get("ROBOFLOW_API_KEY")
if not api_key:
    sys.exit("Set ROBOFLOW_API_KEY in ML-model/.env or as an environment variable.")

rf = Roboflow(api_key=api_key)
dataset = rf.workspace(WORKSPACE).project(PROJECT).version(VERSION).download(
    "yolov8", location=str(DEST)
)
print("Dataset downloaded to:", dataset.location)
