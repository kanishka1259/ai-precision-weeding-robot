import os
import time
import threading
import platform
from collections import deque

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request, render_template_string

# -------------------- CONFIGURATION --------------------

APP_NAME = "AGRI VISION AI"
MODEL_PATH = os.path.expanduser("~/CropWeedAI/models/best.onnx")

CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
MODEL_SIZE = 416
CONFIDENCE_THRESHOLD = 0.35
NMS_THRESHOLD = 0.45

# Raspberry Pi BCM GPIO numbers
MOTOR_IN1 = 23
MOTOR_IN2 = 24
MOTOR_IN3 = 27
MOTOR_IN4 = 22

TRIG_PIN = 5
ECHO_PIN = 6

RELAY_PIN = 17
SERVO_PIN = 12

# Adjust after checking your relay module.
RELAY_ACTIVE_HIGH = True

# Soil ADC values are provisional. Calibrate before enabling Auto.
SOIL_DRY_ADC = 18000
SOIL_WET_ADC = 12000

OBSTACLE_DISTANCE_CM = 20.0
WATERING_SECONDS = 7
SOIL_CHECK_ANGLE = 100   # probe position; matches the dashboard button
MAX_WATERING_CYCLES = 3

# -------------------- FLASK --------------------

app = Flask(__name__)

# -------------------- SHARED STATE --------------------

lock = threading.RLock()

state = {
    "mode": "manual",
    "motor": "stop",
    "distance_cm": None,
    "soil_adc": None,
    "soil_percent": None,
    "soil_status": "Waiting for sensor",
    "crop_count": 0,
    "weed_count": 0,
    "detections": [],
    "camera_ok": False,
    "ai_ok": False,
    "gpio_ok": False,
    "ads_ok": False,
    "servo_angle": 0,
    "relay_on": False,
    "auto_message": "Manual mode",
    "last_error": "",
    "fps": 0,
}

frame_lock = threading.Lock()
latest_frame = None
stop_event = threading.Event()

gpio = None
GPIO = None
ads_channel = None
servo = None
motor_ready = False
relay_ready = False
servo_ready = False
sensor_ready = False
model = None
camera = None

# -------------------- HARDWARE INITIALIZATION --------------------

def initialize_hardware():
    global GPIO, gpio, ads_channel, servo
    global motor_ready, relay_ready, servo_ready, sensor_ready

    try:
        import RPi.GPIO as RGPIO
        GPIO = RGPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        for pin in [MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_IN4,
                    TRIG_PIN, RELAY_PIN]:
            GPIO.setup(pin, GPIO.OUT)

        GPIO.setup(ECHO_PIN, GPIO.IN)

        GPIO.output(TRIG_PIN, GPIO.LOW)
        GPIO.output(RELAY_PIN,
                    GPIO.LOW if RELAY_ACTIVE_HIGH else GPIO.HIGH)

        motor_ready = True
        relay_ready = True

        with lock:
            state["gpio_ok"] = True

    except Exception as e:
        with lock:
            state["last_error"] = "GPIO: " + str(e)

    try:
        import board
        import busio
        from adafruit_ads1x15.ads1115 import ADS1115
        from adafruit_ads1x15.analog_in import AnalogIn
        from adafruit_ads1x15.ads1x15 import Pin

        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS1115(i2c)
        ads_channel = AnalogIn(ads, Pin.A0)
        sensor_ready = True

        with lock:
            state["ads_ok"] = True

    except Exception as e:
        with lock:
            state["last_error"] = "ADS1115: " + str(e)

    try:
        from gpiozero import AngularServo
        from gpiozero.pins.lgpio import LGPIOFactory

        factory = LGPIOFactory()
        servo = AngularServo(
            SERVO_PIN,
            min_angle=0,
            max_angle=180,
            min_pulse_width=0.0005,
            max_pulse_width=0.0025,
            pin_factory=factory
        )
        servo.angle = 0
        servo_ready = True

    except Exception as e:
        with lock:
            state["last_error"] = "Servo: " + str(e)


def motor_stop():
    if not motor_ready:
        with lock:
            state["motor"] = "stop"
        return

    for pin in [MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_IN4]:
        GPIO.output(pin, GPIO.LOW)

    with lock:
        state["motor"] = "stop"


def motor_move(direction):
    if direction not in ["forward", "reverse", "left", "right", "stop"]:
        return False, "Unknown direction"

    if direction == "stop":
        motor_stop()
        return True, "Motors stopped"

    with lock:
        distance = state["distance_cm"]

    # Safety guard applies to every movement command.
    if distance is not None and distance < OBSTACLE_DISTANCE_CM:
        motor_stop()
        return False, "Obstacle detected. Movement blocked."

    if not motor_ready:
        return False, "GPIO is not ready; motors were not activated"

    pins = {
        "forward": (1, 0, 1, 0),
        "reverse": (0, 1, 0, 1),
        "left":    (0, 1, 1, 0),
        "right":   (1, 0, 0, 1),
    }[direction]

    for pin, value in zip(
        [MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_IN4], pins
    ):
        GPIO.output(pin, GPIO.HIGH if value else GPIO.LOW)

    with lock:
        state["motor"] = direction

    return True, "Motor: " + direction


def relay_set(on):
    if not relay_ready:
        return False, "Relay GPIO is not ready"

    level = GPIO.HIGH if on == RELAY_ACTIVE_HIGH else GPIO.LOW
    GPIO.output(RELAY_PIN, level)

    with lock:
        state["relay_on"] = bool(on)

    return True, "Relay " + ("ON" if on else "OFF")


def servo_set(angle):
    angle = max(0, min(180, int(angle)))

    if not servo_ready:
        return False, "Servo is not ready"

    servo.angle = angle

    with lock:
        state["servo_angle"] = angle

    time.sleep(0.5)
    return True, "Servo moved to " + str(angle) + " degrees"


# -------------------- SENSOR READING --------------------

def read_soil():
    if not sensor_ready or ads_channel is None:
        return None

    try:
        raw = int(ads_channel.value)

        # This is a provisional linear display scale.
        # Actual moisture percentage requires calibration.
        span = max(1, SOIL_DRY_ADC - SOIL_WET_ADC)
        percent = int(
            np.clip((SOIL_DRY_ADC - raw) * 100 / span, 0, 100)
        )

        # Higher ADC readings are treated as drier based on
        # the initial readings supplied for this sensor.
        if raw >= SOIL_DRY_ADC:
            status = "DRY"
        elif raw <= SOIL_WET_ADC:
            status = "WET"
        else:
            status = "MOIST"

        with lock:
            state["soil_adc"] = raw
            state["soil_percent"] = percent
            state["soil_status"] = status

        return raw

    except Exception as e:
        with lock:
            state["last_error"] = "Soil reading: " + str(e)
        return None


def read_distance():
    if not motor_ready or GPIO is None:
        return None

    try:
        GPIO.output(TRIG_PIN, GPIO.LOW)
        time.sleep(0.0002)
        GPIO.output(TRIG_PIN, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIG_PIN, GPIO.LOW)

        start = time.monotonic()
        timeout = start + 0.03

        while GPIO.input(ECHO_PIN) == 0:
            start = time.monotonic()
            if start > timeout:
                return None

        end = time.monotonic()
        timeout = end + 0.03

        while GPIO.input(ECHO_PIN) == 1:
            end = time.monotonic()
            if end > timeout:
                return None

        distance = round((end - start) * 17150, 1)

        with lock:
            state["distance_cm"] = distance

        return distance

    except Exception as e:
        with lock:
            state["last_error"] = "Ultrasonic: " + str(e)
        return None


def sensor_worker():
    while not stop_event.is_set():
        try:
            read_distance()
            read_soil()
        except Exception:
            pass
        time.sleep(0.5)


# -------------------- AI MODEL --------------------

def load_model():
    global model

    if not os.path.exists(MODEL_PATH):
        with lock:
            state["last_error"] = "Model file not found: " + MODEL_PATH
        return

    try:
        model = cv2.dnn.readNetFromONNX(MODEL_PATH)
        with lock:
            state["ai_ok"] = True
    except Exception as e:
        with lock:
            state["ai_ok"] = False
            state["last_error"] = "Model load failed: " + str(e)


def letterbox(image, size=416):
    h, w = image.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = int(w * scale), int(h * scale)

    resized = cv2.resize(image, (nw, nh))
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)

    xoff = (size - nw) // 2
    yoff = (size - nh) // 2
    canvas[yoff:yoff + nh, xoff:xoff + nw] = resized

    return canvas, scale, xoff, yoff


def detect_objects(frame):
    if model is None:
        return frame, [], 0, 0

    try:
        original_h, original_w = frame.shape[:2]
        canvas, scale, xoff, yoff = letterbox(frame, MODEL_SIZE)

        blob = cv2.dnn.blobFromImage(
            canvas,
            scalefactor=1 / 255.0,
            size=(MODEL_SIZE, MODEL_SIZE),
            swapRB=True,
            crop=False
        )

        model.setInput(blob)
        output = model.forward()
        output = np.squeeze(output)

        # Model output is expected to be 6 x 3549.
        if output.ndim != 2:
            return frame, [], 0, 0

        if output.shape[0] == 6:
            output = output.T
        elif output.shape[1] != 6:
            return frame, [], 0, 0

        boxes, scores, class_ids = [], [], []

        for row in output:
            cx, cy, bw, bh = map(float, row[:4])
            class_scores = row[4:6]

            class_id = int(np.argmax(class_scores))
            score = float(class_scores[class_id])

            if score < CONFIDENCE_THRESHOLD:
                continue

            # YOLO coordinates are assumed to be in model pixels.
            x1 = (cx - bw / 2 - xoff) / scale
            y1 = (cy - bh / 2 - yoff) / scale
            x2 = (cx + bw / 2 - xoff) / scale
            y2 = (cy + bh / 2 - yoff) / scale

            x1 = int(np.clip(x1, 0, original_w - 1))
            y1 = int(np.clip(y1, 0, original_h - 1))
            x2 = int(np.clip(x2, 0, original_w - 1))
            y2 = int(np.clip(y2, 0, original_h - 1))

            if x2 <= x1 or y2 <= y1:
                continue

            boxes.append([x1, y1, x2 - x1, y2 - y1])
            scores.append(score)
            class_ids.append(class_id)

        indices = cv2.dnn.NMSBoxes(
            boxes, scores, CONFIDENCE_THRESHOLD, NMS_THRESHOLD
        )

        detections = []
        crops = 0
        weeds = 0

        if len(indices):
            for idx in np.array(indices).flatten():
                x, y, w, h = boxes[int(idx)]
                cid = class_ids[int(idx)]
                score = scores[int(idx)]

                label = "Crop" if cid == 0 else "Weed"
                color = (40, 220, 100) if cid == 0 else (30, 80, 255)

                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                cv2.rectangle(
                    frame, (x, max(0, y - 25)),
                    (x + w, y), color, -1
                )
                cv2.putText(
                    frame, f"{label} {score:.0%}",
                    (x + 5, max(15, y - 7)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 0, 0), 2
                )

                detections.append({
                    "class": label,
                    "confidence": round(score, 3),
                    "box": [x, y, w, h]
                })

                if cid == 0:
                    crops += 1
                else:
                    weeds += 1

        with lock:
            state["crop_count"] = crops
            state["weed_count"] = weeds
            state["detections"] = detections

        return frame, detections, crops, weeds

    except Exception as e:
        with lock:
            state["last_error"] = "AI inference: " + str(e)
        return frame, [], 0, 0


# -------------------- CAMERA --------------------

def camera_worker():
    global camera, latest_frame

    camera = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not camera.isOpened():
        with lock:
            state["camera_ok"] = False
            state["last_error"] = "Could not open USB camera"
        return

    with lock:
        state["camera_ok"] = True

    times = deque(maxlen=20)

    while not stop_event.is_set():
        ok, frame = camera.read()

        if not ok:
            time.sleep(0.1)
            continue

        start = time.monotonic()
        frame, detections, crops, weeds = detect_objects(frame)

        with lock:
            auto_mode = state["mode"] == "auto"

        # Safe prototype behavior: move only when crop is detected,
        # stop when there is no crop or an obstacle.
        if auto_mode:
            with lock:
                distance = state["distance_cm"]

            if distance is not None and distance < OBSTACLE_DISTANCE_CM:
                motor_stop()
            elif crops > 0:
                motor_move("forward")
            else:
                motor_stop()

        elapsed = time.monotonic() - start
        times.append(elapsed)

        fps = len(times) / sum(times) if sum(times) else 0

        with lock:
            state["fps"] = round(fps, 1)

        cv2.putText(
            frame, "AGRI VISION AI",
            (15, 30), cv2.FONT_HERSHEY_SIMPLEX,
            0.8, (70, 255, 150), 2
        )
        cv2.putText(
            frame, f"Crops: {crops}   Weeds: {weeds}",
            (15, 60), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (255, 255, 255), 2
        )

        with frame_lock:
            latest_frame = frame.copy()

        time.sleep(0.01)

    if camera is not None:
        camera.release()


def video_generator():
    while not stop_event.is_set():
        with frame_lock:
            frame = None if latest_frame is None else latest_frame.copy()

        if frame is None:
            time.sleep(0.1)
            continue

        ok, buffer = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        )

        if ok:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" +
                buffer.tobytes() + b"\r\n"
            )

        time.sleep(0.03)


# -------------------- AUTOMATIC SOIL CHECK / WATERING --------------------

auto_task_lock = threading.Lock()


def watering_cycle():
    # Prevent two soil-check/watering sequences from running at once.
    if not auto_task_lock.acquire(blocking=False):
        return False, "A soil-check cycle is already running"

    try:
        motor_stop()

        with lock:
            state["auto_message"] = "Checking soil"

        ok, msg = servo_set(SOIL_CHECK_ANGLE)
        if not ok:
            with lock:
                state["auto_message"] = msg
            return False, msg

        time.sleep(1)
        raw = read_soil()

        if raw is None:
            with lock:
                state["auto_message"] = "Soil sensor unavailable; watering stopped"
            servo_set(0)
            return False, "Soil sensor unavailable"

        cycles = 0

        while raw >= SOIL_DRY_ADC and cycles < MAX_WATERING_CYCLES:
            with lock:
                state["auto_message"] = (
                    f"Dry soil: watering cycle {cycles + 1}"
                )

            servo_set(0)

            ok, msg = relay_set(True)
            if not ok:
                servo_set(SOIL_CHECK_ANGLE)
                return False, msg

            # Timed watering. Do not bypass this timeout.
            time.sleep(WATERING_SECONDS)
            relay_set(False)

            cycles += 1
            servo_set(SOIL_CHECK_ANGLE)
            time.sleep(1)
            raw = read_soil()

            if raw is None:
                with lock:
                    state["auto_message"] = "Sensor error; watering stopped"
                break

        relay_set(False)
        servo_set(0)

        if raw is not None and raw >= SOIL_DRY_ADC:
            message = "Safety limit reached; soil still reads dry"
        else:
            message = "Soil check complete; watering not required or moisture detected"

        with lock:
            state["auto_message"] = message

        return True, message

    finally:
        # Always switch relay off after the sequence.
        try:
            relay_set(False)
        except Exception:
            pass
        auto_task_lock.release()


# -------------------- SINGLE-FILE DASHBOARD --------------------

HTML = r"""
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Agri Vision AI</title>
<style>
:root {
 --bg:#07100e; --panel:#101d19; --panel2:#13251f;
 --line:#244337; --green:#4cff9a; --lime:#c6ff74;
 --text:#edf8f1; --muted:#8da99a; --red:#ff6576; --amber:#ffc76b;
}
*{box-sizing:border-box}
body{margin:0;background:radial-gradient(ellipse at top right,#173a2c 0,transparent 42%),var(--bg);
 color:var(--text);font-family:Inter,Segoe UI,Arial,sans-serif}
header{padding:22px 3%;display:flex;align-items:center;justify-content:space-between;
 border-bottom:1px solid var(--line);background:#08140ff0;position:sticky;top:0;z-index:5}
.brand{display:flex;gap:13px;align-items:center}
.brand-mark{width:46px;height:46px;border-radius:14px;background:linear-gradient(140deg,var(--green),#20754d);
 box-shadow:0 0 28px #4cff9a30}
h1{font-size:19px;margin:0;letter-spacing:1.4px}
.subtitle{font-size:11px;color:var(--muted);margin-top:5px;letter-spacing:2px}
.status{font-size:12px;color:var(--green);border:1px solid #326849;padding:9px 13px;border-radius:20px}
.layout{display:grid;grid-template-columns:225px minmax(0,1fr);gap:22px;padding:24px 3%}
nav,.panel,.metric{background:linear-gradient(145deg,#13241e,#0c1713);border:1px solid var(--line);
 border-radius:17px;box-shadow:0 12px 35px #0003}
nav{padding:15px;height:max-content;position:sticky;top:105px}
.nav-label{font-size:10px;letter-spacing:2px;color:var(--muted);padding:12px 10px}
.navbtn{display:block;width:100%;text-align:left;padding:13px 12px;border:0;border-radius:10px;
 color:#a7c0b1;background:transparent;cursor:pointer;margin:3px 0;font-size:13px}
.navbtn.active,.navbtn:hover{color:var(--green);background:#1b382b}
main{min-width:0}
.page{display:none}.page.active{display:block}
.page-head{display:flex;align-items:end;justify-content:space-between;gap:15px;margin-bottom:20px}
.page-head h2{font-size:25px;margin:0 0 7px}.page-head p{margin:0;color:var(--muted);font-size:13px}
.grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:14px;margin-bottom:18px}
.metric{padding:18px;min-width:0}
.metric-label{font-size:11px;letter-spacing:1px;color:var(--muted)}
.metric-value{font-size:27px;font-weight:700;margin:12px 0 7px;overflow-wrap:anywhere}
.metric-foot{font-size:11px;color:var(--muted)}
.green{color:var(--green)}.amber{color:var(--amber)}.red{color:var(--red)}
.panel{padding:19px;margin-bottom:18px;min-width:0}
.panel-title{font-size:14px;font-weight:700;display:flex;justify-content:space-between;gap:10px;margin-bottom:16px}
.panel-title small{font-weight:400;color:var(--muted)}
.two-col{display:grid;grid-template-columns:minmax(0,1.5fr) minmax(260px,1fr);gap:18px}
.camera{background:#020806;border-radius:12px;overflow:hidden;position:relative;min-height:240px}
.camera img{width:100%;height:auto;display:block;min-height:240px;object-fit:contain}
.camera-tag{position:absolute;top:12px;left:12px;background:#06140ddc;border:1px solid #326849;
 padding:7px 10px;border-radius:7px;font-size:10px;color:var(--green);letter-spacing:1px}
.controls{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;max-width:320px;margin:15px auto}
button,select{font:inherit}
.ctrl,.action{border:1px solid #315644;background:#172c22;color:var(--text);padding:12px;border-radius:10px;cursor:pointer}
.ctrl{min-height:47px}.ctrl:hover,.action:hover{border-color:var(--green);color:var(--green)}
.stop{border-color:#7a3340;background:#3a1a20;color:#ff9aa6}
.action{padding:11px 16px;font-size:12px;font-weight:600}
.action.primary{background:#1f4a33;border-color:#2f7a52}
.big{font-size:42px;font-weight:700;margin:6px 0 14px}
.btnrow{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin:12px 0}
.bar{height:8px;background:#1a2b24;border-radius:6px;overflow:hidden;margin:10px 0}
.bar>div{height:100%;width:0;background:linear-gradient(90deg,var(--green),var(--lime));transition:width .3s}
.note{font-size:11px;color:var(--muted);margin-top:10px;line-height:1.5}
pre{background:#06100c;border:1px solid var(--line);border-radius:10px;padding:12px;font-size:11px;
 max-height:160px;overflow:auto;color:var(--lime)}
select{background:#172c22;color:var(--text);border:1px solid #315644;border-radius:8px;padding:8px}
.row{display:flex;justify-content:space-between;align-items:center;gap:10px;margin-bottom:10px}
.mini{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.mini .metric{padding:12px}.mini .metric-value{font-size:18px;margin:8px 0 0}
footer{text-align:center;color:#5c7568;font-size:10px;letter-spacing:1px;padding:28px 0}
@media(max-width:900px){.layout{grid-template-columns:1fr}nav{position:static}
 .grid{grid-template-columns:repeat(2,1fr)}.two-col{grid-template-columns:1fr}}
</style>
</head>
<body>
<header>
 <div class="brand"><div class="brand-mark"></div>
  <div><h1>AGRI VISION AI</h1><div class="subtitle">AUTONOMOUS AGRICULTURE ROBOTICS</div></div></div>
 <div class="status" id="conn">CONNECTING...</div>
</header>
<div class="layout">
<nav>
 <div class="nav-label">ROBOT CONSOLE</div>
 <button class="navbtn active" data-page="mission">Mission Control</button>
 <button class="navbtn" data-page="soil">Soil Intelligence</button>
 <button class="navbtn" data-page="water">Watering Control</button>
 <button class="navbtn" data-page="health">System Health</button>
</nav>
<main>

<section class="page active" id="page-mission">
 <div class="page-head"><div><h2>Mission Control</h2>
  <p>Live field view, AI detection and robot driving in one place.</p></div>
  <div class="status" id="modeTag">MANUAL MODE</div></div>
 <div class="grid">
  <div class="metric"><div class="metric-label">CROP DETECTIONS</div><div class="metric-value green" id="crops">0</div><div class="metric-foot">Current camera frame</div></div>
  <div class="metric"><div class="metric-label">WEED DETECTIONS</div><div class="metric-value amber" id="weeds">0</div><div class="metric-foot">Current camera frame</div></div>
  <div class="metric"><div class="metric-label">OBSTACLE DISTANCE</div><div class="metric-value" id="dist">--</div><div class="metric-foot">Ultrasonic sensor</div></div>
  <div class="metric"><div class="metric-label">SOIL CONDITION</div><div class="metric-value green" id="soilTop">--</div><div class="metric-foot" id="soilTopAdc">ADC: --</div></div>
 </div>
 <div class="two-col">
  <div class="panel">
   <div class="panel-title">LIVE AI CAMERA <small id="fps">-- FPS</small></div>
   <div class="camera"><div class="camera-tag">LIVE FIELD FEED</div><img src="/video" alt="Live camera"></div>
   <div class="note">GREEN = CROP &nbsp; | &nbsp; RED = WEED</div>
   <div class="panel-title" style="margin-top:18px">AI DETECTION RESULTS</div>
   <pre id="dets">[]</pre>
  </div>
  <div>
   <div class="panel">
    <div class="panel-title">ROBOT DRIVE CONTROL <small>BCM GPIO</small></div>
    <div class="row"><span class="metric-foot">DRIVING MODE</span>
     <select id="mode"><option value="manual">MANUAL</option><option value="auto">AUTONOMOUS</option></select></div>
    <div class="controls">
     <span></span><button class="ctrl" data-dir="forward">FORWARD</button><span></span>
     <button class="ctrl" data-dir="left">LEFT</button><button class="ctrl stop" data-dir="stop">STOP</button><button class="ctrl" data-dir="right">RIGHT</button>
     <span></span><button class="ctrl" data-dir="reverse">REVERSE</button><span></span>
    </div>
    <div class="note">Use the live camera to see the field while driving. Stop immediately if the robot behaves unexpectedly.</div>
   </div>
   <div class="panel">
    <div class="panel-title">LIVE TELEMETRY</div>
    <div class="mini">
     <div class="metric"><div class="metric-label">MOTOR STATE</div><div class="metric-value green" id="motor">STOP</div></div>
     <div class="metric"><div class="metric-label">PATH STATUS</div><div class="metric-value" id="path">CLEAR</div></div>
     <div class="metric"><div class="metric-label">CAMERA</div><div class="metric-value" id="cam">--</div></div>
     <div class="metric"><div class="metric-label">AI MODEL</div><div class="metric-value" id="ai">--</div></div>
    </div>
    <div class="row" style="margin-top:14px"><span class="metric-foot">DISTANCE SAFETY METER</span><span class="metric-foot" id="distTxt">--</span></div>
    <div class="bar"><div id="distBar"></div></div>
    <div class="panel-title" style="margin-top:16px">AUTONOMOUS ACTIVITY</div>
    <div class="note" id="autoMsg">Manual mode</div>
   </div>
  </div>
 </div>
</section>

<section class="page" id="page-soil">
 <div class="page-head"><div><h2>Soil Intelligence</h2><p>ADS1115 analog moisture monitoring.</p></div></div>
 <div class="grid">
  <div class="metric"><div class="metric-label">RAW ADC VALUE</div><div class="metric-value" id="adc">--</div></div>
  <div class="metric"><div class="metric-label">ESTIMATED MOISTURE</div><div class="metric-value green" id="pct">--</div></div>
  <div class="metric"><div class="metric-label">SOIL CLASS</div><div class="metric-value" id="cls">--</div></div>
  <div class="metric"><div class="metric-label">ADS1115</div><div class="metric-value" id="ads">--</div></div>
 </div>
 <div class="panel"><div class="panel-title">MOISTURE MONITOR</div>
  <div class="big green" id="pct2">0%</div><div class="bar"><div id="pctBar"></div></div>
  <div class="note">The percentage is a provisional estimate. Calibrate readings in your actual dry and wet soil before using automated watering.</div>
  <p><button class="action primary" id="soilRun">RUN SOIL CHECK</button></p></div>
</section>

<section class="page" id="page-water">
 <div class="page-head"><div><h2>Watering Control</h2><p>Servo positioning and timed relay output.</p></div></div>
 <div class="two-col" style="grid-template-columns:1fr 1fr">
  <div class="panel"><div class="panel-title">SERVO POSITION</div>
   <div class="big green" id="servo">0&deg;</div>
   <div class="btnrow"><button class="action" data-servo="0">NORMAL &middot; 0&deg;</button>
    <button class="action" data-servo="100">SOIL CHECK &middot; 100&deg;</button></div>
   <div class="note">Check that the servo can move freely and does not force the probe against an obstruction.</div></div>
  <div class="panel"><div class="panel-title">RELAY OUTPUT</div>
   <div class="big" id="relay">OFF</div>
   <div class="btnrow"><button class="action primary" data-relay="1">RELAY ON</button>
    <button class="action stop" data-relay="0">RELAY OFF</button></div>
   <div class="note">Keep the pump disconnected during initial tests. Verify relay polarity and load ratings before connecting a pump.</div></div>
 </div>
 <div class="panel"><div class="panel-title">SOIL CHECK SEQUENCE</div>
  <div class="note" id="autoMsg2">Manual control enabled</div>
  <p><button class="action primary" id="soilRun2">START SOIL CHECK</button></p>
  <div class="note">If the provisional dry threshold is met, the relay may turn on for 7 seconds per cycle, with a maximum of 3 cycles. Verify the thresholds first.</div></div>
</section>

<section class="page" id="page-health">
 <div class="page-head"><div><h2>System Health</h2><p>Hardware and software readiness.</p></div></div>
 <div class="grid">
  <div class="metric"><div class="metric-label">GPIO</div><div class="metric-value" id="h_gpio">--</div></div>
  <div class="metric"><div class="metric-label">CAMERA</div><div class="metric-value" id="h_cam">--</div></div>
  <div class="metric"><div class="metric-label">AI MODEL</div><div class="metric-value" id="h_ai">--</div></div>
  <div class="metric"><div class="metric-label">ADS1115</div><div class="metric-value" id="h_ads">--</div></div>
 </div>
 <div class="panel"><div class="panel-title">LAST ERROR</div><pre id="err">none</pre></div>
</section>

<footer>AGRI VISION AI &middot; EDGE AGRICULTURE ROBOTICS</footer>
</main>
</div>
<script>
const $=id=>document.getElementById(id);
function post(url,body){return fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},
 body:JSON.stringify(body||{})}).then(r=>r.json()).catch(()=>({ok:false,message:'Request failed'}));}
document.querySelectorAll('.navbtn').forEach(b=>b.onclick=()=>{
 document.querySelectorAll('.navbtn').forEach(x=>x.classList.remove('active'));b.classList.add('active');
 document.querySelectorAll('.page').forEach(p=>p.classList.remove('active'));
 $('page-'+b.dataset.page).classList.add('active');});
document.querySelectorAll('[data-dir]').forEach(b=>b.onclick=()=>post('/api/move',{direction:b.dataset.dir}).then(r=>{if(!r.ok)$('autoMsg').textContent=r.message;}));
document.querySelectorAll('[data-servo]').forEach(b=>b.onclick=()=>post('/api/servo',{angle:+b.dataset.servo}));
document.querySelectorAll('[data-relay]').forEach(b=>b.onclick=()=>post('/api/relay',{on:b.dataset.relay==='1'}));
$('mode').onchange=e=>post('/api/mode',{mode:e.target.value});
['soilRun','soilRun2'].forEach(i=>$(i).onclick=()=>post('/api/soil_check'));
const ok=v=>v?'READY':'OFFLINE';
function refresh(){
 fetch('/api/status').then(r=>r.json()).then(s=>{
  $('conn').textContent='SYSTEM CONNECTED';
  $('crops').textContent=s.crop_count;$('weeds').textContent=s.weed_count;
  const d=s.distance_cm;$('dist').textContent=d==null?'--':d+' cm';$('distTxt').textContent=d==null?'--':d+' cm';
  $('distBar').style.width=d==null?'0%':Math.min(100,d)+'%';
  const blocked=d!=null&&d<__OBST__;
  $('path').textContent=blocked?'OBSTACLE':'CLEAR';$('path').className='metric-value '+(blocked?'red':'green');
  $('soilTop').textContent=s.soil_status;$('soilTopAdc').textContent='ADC: '+(s.soil_adc==null?'--':s.soil_adc);
  $('fps').textContent=s.fps+' FPS';$('dets').textContent=JSON.stringify(s.detections,null,1);
  $('motor').textContent=s.motor.toUpperCase();$('cam').textContent=ok(s.camera_ok);$('ai').textContent=ok(s.ai_ok);
  $('modeTag').textContent=s.mode.toUpperCase()+' MODE';
  if(document.activeElement!==$('mode'))$('mode').value=s.mode;
  $('autoMsg').textContent=s.auto_message;$('autoMsg2').textContent=s.auto_message;
  $('adc').textContent=s.soil_adc==null?'--':s.soil_adc;
  const pc=s.soil_percent==null?null:s.soil_percent;
  $('pct').textContent=pc==null?'--':pc+'%';$('pct2').textContent=pc==null?'--':pc+'%';$('pctBar').style.width=(pc||0)+'%';
  $('cls').textContent=s.soil_status;$('ads').textContent=ok(s.ads_ok);
  $('servo').innerHTML=s.servo_angle+'&deg;';$('relay').textContent=s.relay_on?'ON':'OFF';
  $('h_gpio').textContent=ok(s.gpio_ok);$('h_cam').textContent=ok(s.camera_ok);
  $('h_ai').textContent=ok(s.ai_ok);$('h_ads').textContent=ok(s.ads_ok);
  $('err').textContent=s.last_error||'none';
 }).catch(()=>{$('conn').textContent='DISCONNECTED';});
}
refresh();setInterval(refresh,1000);
</script>
</body>
</html>
"""

# -------------------- ROUTES --------------------

@app.route("/")
def index():
    return HTML.replace("__OBST__", str(OBSTACLE_DISTANCE_CM))


@app.route("/video")
def video():
    return Response(
        video_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/api/status")
def api_status():
    with lock:
        return jsonify(dict(state))


@app.route("/api/mode", methods=["POST"])
def api_mode():
    mode = (request.get_json(silent=True) or {}).get("mode")
    if mode not in ("manual", "auto"):
        return jsonify(ok=False, message="Unknown mode"), 400

    motor_stop()
    with lock:
        state["mode"] = mode
        state["auto_message"] = (
            "Autonomous mode: driving only while a crop is detected"
            if mode == "auto" else "Manual control enabled"
        )
    return jsonify(ok=True, message="Mode: " + mode)


@app.route("/api/move", methods=["POST"])
def api_move():
    direction = (request.get_json(silent=True) or {}).get("direction")

    with lock:
        auto_mode = state["mode"] == "auto"

    # Drive buttons are ignored in auto mode, except STOP.
    if auto_mode and direction != "stop":
        return jsonify(ok=False, message="Switch to manual mode to drive")

    ok, msg = motor_move(direction)
    return jsonify(ok=ok, message=msg)


@app.route("/api/servo", methods=["POST"])
def api_servo():
    try:
        angle = int((request.get_json(silent=True) or {}).get("angle"))
    except (TypeError, ValueError):
        return jsonify(ok=False, message="Invalid angle"), 400

    ok, msg = servo_set(angle)
    return jsonify(ok=ok, message=msg)


@app.route("/api/relay", methods=["POST"])
def api_relay():
    on = bool((request.get_json(silent=True) or {}).get("on"))
    ok, msg = relay_set(on)
    return jsonify(ok=ok, message=msg)


@app.route("/api/soil_check", methods=["POST"])
def api_soil_check():
    if auto_task_lock.locked():
        return jsonify(ok=False, message="A soil-check cycle is already running")

    threading.Thread(target=watering_cycle, daemon=True).start()
    return jsonify(ok=True, message="Soil check started")


# -------------------- MAIN --------------------

def cleanup():
    stop_event.set()
    try:
        motor_stop()
        relay_set(False)
    except Exception:
        pass
    try:
        if servo is not None:
            servo.close()
    except Exception:
        pass
    try:
        if GPIO is not None:
            GPIO.cleanup()
    except Exception:
        pass


if __name__ == "__main__":
    initialize_hardware()
    load_model()

    threading.Thread(target=sensor_worker, daemon=True).start()
    threading.Thread(target=camera_worker, daemon=True).start()

    try:
        app.run(host="0.0.0.0", port=5000, threaded=True, use_reloader=False)
    finally:
        cleanup()
