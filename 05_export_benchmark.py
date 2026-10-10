"""Step 5: export the model for the Raspberry Pi and measure speed.

Run here for a first estimate, then copy this script, config.py, the exported model and a few
test images to the Pi and run `python 05_export_benchmark.py --no-export --model <path>` there.
"""
import argparse
import time

from ultralytics import YOLO

import config


def benchmark(model_path, imgsz, images, runs=40):
    model = YOLO(str(model_path), task="detect")
    for _ in range(5):  # warm-up
        model.predict(images[0], imgsz=imgsz, device="cpu", verbose=False)
    total = 0.0
    for i in range(runs):
        t = time.perf_counter()
        model.predict(images[i % len(images)], imgsz=imgsz, device="cpu", verbose=False)
        total += time.perf_counter() - t
    ms = total / runs * 1000
    return ms, 1000 / ms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=str(config.BEST_WEIGHTS))
    ap.add_argument("--sizes", type=int, nargs="+", default=[320, 416])
    ap.add_argument("--formats", nargs="+", default=["onnx", "ncnn"])
    ap.add_argument("--no-export", action="store_true", help="only benchmark --model")
    ap.add_argument("--model", help="exported model path to benchmark (with --no-export)")
    ap.add_argument("--images", default=str(config.PREPARED_DIR / "images" / "test"))
    args = ap.parse_args()

    from pathlib import Path
    images = sorted(str(p) for p in Path(args.images).iterdir() if p.suffix.lower() in config.IMAGE_EXTS)[:20]

    rows = []
    if args.no_export:
        for sz in args.sizes:
            ms, fps = benchmark(args.model, sz, images)
            rows.append((args.model, sz, ms, fps))
    else:
        base = YOLO(args.weights)
        rows.append(("best.pt (PyTorch)",) + (config.IMG_SIZE,) + benchmark(args.weights, config.IMG_SIZE, images))
        for fmt in args.formats:
            for sz in args.sizes:
                try:
                    path = base.export(format=fmt, imgsz=sz)
                    ms, fps = benchmark(path, sz, images)
                    rows.append((f"{fmt}", sz, ms, fps))
                except Exception as e:  # e.g. missing optional export dependency
                    print(f"export {fmt}@{sz} failed: {e}")

    print("\n| Model | imgsz | ms/image | FPS |\n| :--- | :-: | :-: | :-: |")
    for name, sz, ms, fps in rows:
        print(f"| {name} | {sz} | {ms:.1f} | {fps:.1f} |")
    print("\nThis is this computer's CPU. Re-run on the Raspberry Pi for the real numbers.")


if __name__ == "__main__":
    main()
