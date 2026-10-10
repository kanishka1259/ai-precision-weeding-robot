"""Step 4: evaluate the trained model on the held-out test split and save predictions."""
import argparse
import random

from ultralytics import YOLO

import config


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=str(config.BEST_WEIGHTS))
    ap.add_argument("--imgsz", type=int, default=config.IMG_SIZE)
    args = ap.parse_args()

    model = YOLO(args.weights)
    m = model.val(data=str(config.DATA_YAML), split="test", imgsz=args.imgsz, device="cpu",
                  project=str(config.OUTPUTS_DIR), name="eval", exist_ok=True)

    lines = [
        "# Test-set results", "",
        f"weights: `{args.weights}`, imgsz: {args.imgsz}", "",
        "| Class | Precision | Recall | mAP@0.5 | mAP@0.5:0.95 |", "| :--- | :-: | :-: | :-: | :-: |",
    ]
    for i, c in enumerate(m.box.ap_class_index):
        p, r, ap50, ap = m.box.class_result(i)
        lines.append(f"| {model.names[int(c)]} | {p:.3f} | {r:.3f} | {ap50:.3f} | {ap:.3f} |")
    lines.append(f"| **all** | {m.box.mp:.3f} | {m.box.mr:.3f} | {m.box.map50:.3f} | {m.box.map:.3f} |")
    report = "\n".join(lines)
    print("\n" + report)
    config.OUTPUTS_DIR.mkdir(exist_ok=True)
    (config.OUTPUTS_DIR / "results.md").write_text(report + "\n")

    # visual check on a few test images
    test_imgs = sorted((config.PREPARED_DIR / "images" / "test").iterdir())
    random.seed(config.SEED)
    sample = random.sample(test_imgs, min(12, len(test_imgs)))
    model.predict(sample, imgsz=args.imgsz, device="cpu", conf=0.25, save=True,
                  project=str(config.OUTPUTS_DIR), name="predictions", exist_ok=True)
    print("confusion matrix / curves:", config.OUTPUTS_DIR / "eval")
    print("prediction images:", config.OUTPUTS_DIR / "predictions")


if __name__ == "__main__":
    main()
