"""Step 1: audit the raw datasets (read-only) and save sample images with boxes drawn."""
import random
from collections import Counter

import cv2

import config

COLORS = {0: (0, 200, 0), 1: (0, 0, 255)}  # BGR: crop = green, weed = red


def read_labels(path):
    """Return (boxes, problems). boxes = [(cls, x, y, w, h)]."""
    boxes, problems = [], []
    for n, line in enumerate(path.read_text().splitlines(), 1):
        parts = line.split()
        if not parts:
            continue
        try:
            cls = int(parts[0])
            x, y, w, h = map(float, parts[1:])
            if len(parts) != 5 or not all(0 <= v <= 1 for v in (x, y, w, h)):
                raise ValueError
        except ValueError:
            problems.append(f"{path.name}:{n} bad line '{line}'")
            continue
        boxes.append((cls, x, y, w, h))
    return boxes, problems


def audit(name, pairs):
    """pairs: list of (image_path, label_path)."""
    print(f"\n=== {name}: {len(pairs)} images ===")
    cls_count, per_image, empty, problems, sizes = Counter(), [], 0, [], Counter()
    for img, lbl in pairs:
        boxes, p = read_labels(lbl)
        problems += p
        per_image.append(len(boxes))
        empty += not boxes
        cls_count.update(b[0] for b in boxes)
        im = cv2.imread(str(img))
        if im is None:
            problems.append(f"{img.name} cannot be read")
        else:
            sizes[(im.shape[1], im.shape[0])] += 1
    print("boxes per class:", dict(sorted(cls_count.items())))
    print(f"boxes per image: min {min(per_image)}, max {max(per_image)}, "
          f"mean {sum(per_image) / len(per_image):.2f}; images with no boxes: {empty}")
    print("image sizes (WxH):", dict(sizes.most_common(5)))
    print("problems:", len(problems))
    for p in problems[:10]:
        print("  ", p)


def save_samples(pairs, out_dir, n=12):
    out_dir.mkdir(parents=True, exist_ok=True)
    random.seed(config.SEED)
    for img, lbl in random.sample(pairs, min(n, len(pairs))):
        im = cv2.imread(str(img))
        if im is None:
            continue
        h, w = im.shape[:2]
        for cls, x, y, bw, bh in read_labels(lbl)[0]:
            p1 = (int((x - bw / 2) * w), int((y - bh / 2) * h))
            p2 = (int((x + bw / 2) * w), int((y + bh / 2) * h))
            cv2.rectangle(im, p1, p2, COLORS.get(cls, (255, 0, 0)), 2)
        cv2.imwrite(str(out_dir / img.name), im)
    print("sample images saved to", out_dir)


def main():
    # Kaggle: flat folder of image + label pairs
    pairs = []
    for img in sorted(config.KAGGLE_DIR.iterdir()):
        if img.suffix.lower() in config.IMAGE_EXTS:
            lbl = img.with_suffix(".txt")
            if lbl.exists():
                pairs.append((img, lbl))
            else:
                print("no label for", img.name)
    audit("Kaggle (0=crop, 1=weed)", pairs)
    save_samples(pairs, config.OUTPUTS_DIR / "samples_kaggle")

    # Roboflow: weed-only, pre-split
    if config.ROBOFLOW_DIR.exists():
        rf_pairs = []
        for split in ("train", "valid", "test"):
            for img in sorted((config.ROBOFLOW_DIR / split / "images").iterdir()):
                lbl = config.ROBOFLOW_DIR / split / "labels" / (img.stem + ".txt")
                if lbl.exists():
                    rf_pairs.append((img, lbl))
        audit("Roboflow (class 0 = weed only)", rf_pairs)
        save_samples(rf_pairs, config.OUTPUTS_DIR / "samples_roboflow", n=6)

    print("\nOpen the sample folders and check the boxes sit on the right plants "
          "(green = crop, red = weed; Roboflow's single class is drawn green).")


if __name__ == "__main__":
    main()
