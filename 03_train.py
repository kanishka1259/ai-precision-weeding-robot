"""Step 3: train YOLOv8n on CPU.

Smoke test (check speed first):  python 03_train.py --smoke
Full run:                        python 03_train.py
"""
import argparse

from ultralytics import YOLO

import config


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--imgsz", type=int, default=config.IMG_SIZE)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--model", default="yolov8n.pt")
    ap.add_argument("--smoke", action="store_true",
                    help="1 epoch on 10%% of the data, to measure time per epoch")
    ap.add_argument("--resume", action="store_true", help="resume the last interrupted run")
    args = ap.parse_args()

    if args.resume:
        YOLO(str(config.RUNS_DIR / config.RUN_NAME / "weights" / "last.pt")).train(resume=True)
        return

    model = YOLO(args.model)  # pretrained on COCO; downloaded on first use
    model.train(
        data=str(config.DATA_YAML),
        epochs=1 if args.smoke else args.epochs,
        fraction=0.1 if args.smoke else 1.0,
        imgsz=args.imgsz,
        batch=args.batch,
        workers=args.workers,
        device="cpu",
        patience=15,            # stop early if val mAP stops improving
        cache=True,             # keep resized images in RAM
        seed=config.SEED,
        project=str(config.RUNS_DIR),
        name="smoke" if args.smoke else config.RUN_NAME,
        exist_ok=True,
        # augmentation tuned for field images
        fliplr=0.5, flipud=0.2, hsv_h=0.015, hsv_s=0.6, hsv_v=0.4,
        degrees=10, scale=0.4, mosaic=1.0, close_mosaic=5,
    )


if __name__ == "__main__":  # required on Windows (dataloader worker processes)
    main()
