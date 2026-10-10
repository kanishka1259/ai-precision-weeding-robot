"""Step 2: build the YOLO dataset (train/val/test) and crop_weed.yaml.

Default: Kaggle crop+weed data only, split 70/20/10 (stratified by which classes
appear in each image).

--add-roboflow-weeds also adds the Roboflow weed-only images to the TRAIN split
(class 0 -> 1 "weed"). Their crops are probably unlabelled, which can teach the model
that crops are background, so only use this if crop recall stays good.
"""
import argparse
import random
import shutil
from collections import defaultdict

import yaml

import config


def kaggle_pairs():
    pairs = []
    for img in sorted(config.KAGGLE_DIR.iterdir()):
        lbl = img.with_suffix(".txt")
        if img.suffix.lower() in config.IMAGE_EXTS and lbl.exists():
            classes = {l.split()[0] for l in lbl.read_text().splitlines() if l.strip()}
            if classes:  # skip images with no valid boxes
                pairs.append((img, lbl, frozenset(classes)))
    return pairs


def stratified_split(pairs):
    groups = defaultdict(list)
    for p in pairs:
        groups[p[2]].append(p)
    rng = random.Random(config.SEED)
    out = {"train": [], "val": [], "test": []}
    for items in groups.values():
        rng.shuffle(items)
        n = len(items)
        n_train = round(n * config.SPLIT[0])
        n_val = round(n * config.SPLIT[1])
        out["train"] += items[:n_train]
        out["val"] += items[n_train:n_train + n_val]
        out["test"] += items[n_train + n_val:]
    return out


def copy_pair(img, label_text, split, prefix=""):
    (config.PREPARED_DIR / "images" / split).mkdir(parents=True, exist_ok=True)
    (config.PREPARED_DIR / "labels" / split).mkdir(parents=True, exist_ok=True)
    name = prefix + img.name
    shutil.copy2(img, config.PREPARED_DIR / "images" / split / name)
    (config.PREPARED_DIR / "labels" / split / (prefix + img.stem + ".txt")).write_text(label_text)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--add-roboflow-weeds", action="store_true")
    args = ap.parse_args()

    if config.PREPARED_DIR.exists():
        shutil.rmtree(config.PREPARED_DIR)

    splits = stratified_split(kaggle_pairs())
    for split, items in splits.items():
        for img, lbl, _ in items:
            copy_pair(img, lbl.read_text(), split)
        print(f"{split}: {len(items)} Kaggle images")

    if args.add_roboflow_weeds:
        added = 0
        for sub in ("train", "valid", "test"):
            for img in sorted((config.ROBOFLOW_DIR / sub / "images").iterdir()):
                lbl = config.ROBOFLOW_DIR / sub / "labels" / (img.stem + ".txt")
                if not lbl.exists():
                    continue
                lines = [l.split() for l in lbl.read_text().splitlines() if l.strip()]
                text = "\n".join(" ".join(["1"] + p[1:]) for p in lines) + "\n"  # weed -> 1
                copy_pair(img, text, "train", prefix="rf_")
                added += 1
        print(f"train: +{added} Roboflow weed images")

    data = {
        "path": str(config.PREPARED_DIR.resolve()),
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",
        "names": dict(enumerate(config.CLASS_NAMES)),
    }
    config.DATA_YAML.write_text(yaml.safe_dump(data, sort_keys=False))
    print("wrote", config.DATA_YAML)


if __name__ == "__main__":
    main()
