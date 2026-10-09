# Crop vs. Weed Detection Model (YOLO)

Vision component of **AI-Based Autonomous Plant Identification, Weed Detection and Precision Spraying Robot using YOLO and Raspberry Pi**
Course project for 23CSE472 – Artificial Intelligence and Robotics, Amrita Vishwa Vidyapeetham (School of Computing, CSE).

## Project Overview

Small and medium farms usually spray fertilizer and weedicide uniformly across a whole field. This wastes chemicals, raises input costs and harms the environment. This project builds a low-cost 4WD ground robot that drives along a row of plants, uses a camera to tell **crops** from **weeds**, and sprays only what it detects: **fertilizer on crops, weedicide on weeds**.

This folder holds the machine-learning part of that system: the lightweight YOLO object detector that runs on the robot's Raspberry Pi 4.

## Role of the Model

| Item | Detail |
| :--- | :--- |
| Task | Object detection with two classes: `crop` and `weed` |
| Input | Live USB webcam frames (about 640 px wide) |
| Output | Bounding boxes, class labels and confidences per frame |
| Target hardware | Raspberry Pi 4, CPU only (no GPU) |
| Target speed | About 5–10 FPS |
| Framework | Ultralytics YOLO (YOLOv5 / YOLOv8 nano or tiny variant) |
| Training | Offline on a separate GPU machine; trained weights are copied to the Pi |

## Why YOLO

Several object detection models were compared on the weed dataset. YOLO gave the best balance of accuracy and computational cost. RF-DETR Nano scored slightly higher in accuracy, but YOLO needs far less storage, RAM and VRAM, which suits the Raspberry Pi-based robot.

## How the Detections Are Used

For each detection, the bounding-box centre `(u, v)` is used downstream:

1. **Target localization (pinhole camera model).** With camera height `h_c`, pitch `α`, focal lengths `f_x, f_y` and optical centre `(c_x, c_y)`:
   - `x_c = (u − c_x) / f_x`, `y_c = (v − c_y) / f_y`
   - `x_p = h_c / tan(α + arctan(y_c))`, `y_p = x_p · x_c`
2. **Nozzle steering.** `θ_s = clamp(arctan(y_p / d_s), −45°, +45°)` drives the MG996R servo.
3. **Spray timing.** `Δt_act = D_sep / v − Δt_sys`, where `Δt_sys` is the sum of vision, communication, relay and fluid delays.
4. **Spray action.** The class label picks the liquid (`crop` → fertilizer, `weed` → weedicide). The relay switches the pump, and spray duration grows with bounding-box area.

The full derivation is in Section 2 of the review document.

## Simplified Control Loop

```python
import cv2
from ultralytics import YOLO
from gpiozero import AngularServo, OutputDevice

model = YOLO("crop_weed_yolo.pt")          # trained lightweight YOLO weights
servo = AngularServo(18, min_angle=0, max_angle=180)
pump_relay = OutputDevice(23, active_high=True, initial_value=False)

CAMERA_FOV_DEG = 60
FRAME_WIDTH = 640

def bbox_to_servo_angle(x_center_px):
    offset_ratio = (x_center_px - FRAME_WIDTH / 2) / (FRAME_WIDTH / 2)
    return 90 + offset_ratio * (CAMERA_FOV_DEG / 2)   # 90 deg = nozzle centred

def spray_duration_for(box_area_px):
    return min(0.3 + box_area_px / 50000.0, 1.5)      # seconds, capped

cap = cv2.VideoCapture(0)
while True:
    ok, frame = cap.read()
    if not ok:
        continue
    for box in model(frame, verbose=False)[0].boxes:
        label = model.names[int(box.cls[0])]           # "crop" or "weed"
        x1, y1, x2, y2 = box.xyxy[0]
        servo.angle = bbox_to_servo_angle((x1 + x2) / 2)
        pump_relay.on()
        cv2.waitKey(int(spray_duration_for((x2 - x1) * (y2 - y1)) * 1000))
        pump_relay.off()
```

This is the simplified listing from the review document. The `label` is not yet used to choose between the fertilizer and weedicide lines. That selection is part of the integration work still to do.

## Setup

```bash
pip install ultralytics opencv-python numpy matplotlib
# On the Raspberry Pi, also:
pip install gpiozero RPi.GPIO adafruit-circuitpython-ads1x15
```

### Training (GPU machine)

```bash
yolo detect train data=crop_weed.yaml model=yolov8n.pt imgsz=640 epochs=100
```

The command above is a starting template. `crop_weed.yaml` (dataset paths and the `crop` / `weed` class names) still needs to be created, and the epochs and image size should be tuned to the dataset. Copy the resulting `best.pt` to the Pi as `crop_weed_yolo.pt`.

### Inference (Raspberry Pi)

```bash
yolo detect predict model=crop_weed_yolo.pt source=0
```

## Status

| Item | Status |
| :--- | :--- |
| Model comparison on the weed dataset | Done: YOLO selected (see above) |
| YOLO crop/weed training | In progress |
| Drive, sensing and spraying subsystems | Bench-tested individually |
| Full detect-aim-spray loop on the robot | Under integration and testing |

No final accuracy or FPS figures are recorded yet.

## Future Work

- Finish training and validation on a larger, more varied labelled dataset.
- Tune the bearing-to-servo mapping and spray-duration function with real bench-test data.
- Test under different lighting and plant-spacing conditions.
- Compare YOLO against other lightweight detectors (e.g. MobileNet-SSD) on the Pi 4.
- Consider a Coral USB Accelerator / Edge TPU or Raspberry Pi AI Camera if the frame rate is too low.
- Measure the chemical savings against a blanket-spraying baseline.

## Hardware Used

Raspberry Pi 4, USB webcam, 4WD chassis with L298N driver, MG996R servo (nozzle pan), 12 V mini pump with 5 V relay, HC-SR04 ultrasonic sensor, capacitive soil-moisture sensor via ADS1115, 12 V battery with LM2596 buck converter.

## Team

| Name | Roll No. |
| :--- | :--- |
| Manvitha Dungi | CB.SC.U4CSE23318 |
| Kanishka S | CB.SC.U4CSE23328 |
| Leela M | CB.SC.U4CSE23336 |
| Jaswita Manduva | CB.SC.U4CSE23430 |
| Bharath | CB.SC.U4CSE23609 |
| Pranav | CB.SC.U4CSE23611 |

## References

1. J. Redmon, S. Divvala, R. Girshick, A. Farhadi, "You Only Look Once: Unified, Real-Time Object Detection," CVPR, 2016.
2. R. Siegwart, I. R. Nourbakhsh, D. Scaramuzza, *Introduction to Autonomous Mobile Robots*, 2nd ed., MIT Press, 2011.
3. A. Wang, W. Zhang, X. Wei, "A review on weed detection using ground-based machine vision and image processing techniques," *Computers and Electronics in Agriculture*, vol. 158, pp. 226–240, 2019.
