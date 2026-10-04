#!/usr/bin/env python3
"""
===============================================================================
PHASE 3 (STEP 1): DATA COLLECTION FOR MACHINE LEARNING AUTONOMOUS DRIVING
===============================================================================
Connects to the ESP32-CAM live video stream, lets you drive the 4WD car using
keyboard controls, sends real-time commands to the car, and saves labeled camera
frames into `dataset/<COMMAND>/` + `dataset/driving_log.csv` for CNN training.

Usage with real ESP32-CAM:
    python3 1_collect_data.py --ip 192.168.4.1

Usage in simulation mode (generates sample road dataset to test ML training):
    python3 1_collect_data.py --simulate --samples 160
===============================================================================
"""

import argparse
import csv
import os
import threading
import time
from datetime import datetime

import numpy as np
import requests
from PIL import Image, ImageDraw

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


VALID_CLASSES = ["F", "G", "I", "L", "R", "S"]
KEY_TO_CMD = {
    ord("w"): "F",  # Forward straight
    ord("W"): "F",
    ord("q"): "G",  # Smooth curve Left
    ord("Q"): "G",
    ord("e"): "I",  # Smooth curve Right
    ord("E"): "I",
    ord("a"): "L",  # Sharp spin Left
    ord("A"): "L",
    ord("d"): "R",  # Sharp spin Right
    ord("D"): "R",
    ord("s"): "S",  # Stop
    ord("S"): "S",
    ord(" "): "S",  # Spacebar = Stop
}


def send_car_command_async(esp_ip: str, cmd: str) -> None:
    """Send command to ESP32-CAM /cmd endpoint in a background thread."""
    def _worker():
        try:
            requests.get(f"http://{esp_ip}/cmd", params={"c": cmd}, timeout=0.35)
        except requests.RequestException:
            pass

    threading.Thread(target=_worker, daemon=True).start()


def ensure_dataset_dirs(dataset_dir: str) -> str:
    for cls in VALID_CLASSES:
        os.makedirs(os.path.join(dataset_dir, cls), exist_ok=True)
    csv_path = os.path.join(dataset_dir, "driving_log.csv")
    if not os.path.exists(csv_path):
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "image_path", "command"])
    return csv_path


def save_labeled_frame(img_rgb: np.ndarray, cmd: str, dataset_dir: str, csv_path: str) -> str:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    rel_path = os.path.join(cmd, f"frame_{ts}.jpg")
    full_path = os.path.join(dataset_dir, rel_path)
    Image.fromarray(img_rgb).save(full_path, quality=90)
    with open(csv_path, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([ts, rel_path, cmd])
    return full_path


def generate_synthetic_road_frame(cmd: str, width: int = 320, height: int = 240) -> np.ndarray:
    """Create a realistic synthetic camera view of a floor track/corridor for testing."""
    img = Image.new("RGB", (width, height), (35, 42, 52))
    draw = ImageDraw.Draw(img)

    # Horizon / ceiling vs floor
    horizon_y = int(height * 0.35)
    draw.rectangle([0, 0, width, horizon_y], fill=(65, 75, 90))
    draw.rectangle([0, horizon_y, width, height], fill=(48, 54, 64))

    # Shift road center based on steering command needed to stay on road
    rng = np.random.default_rng()
    jitter = int(rng.integers(-12, 13))
    shift_map = {"F": 0, "G": -38, "I": 38, "L": -75, "R": 75, "S": 0}
    top_center_x = width // 2 + shift_map.get(cmd, 0) + jitter
    bottom_center_x = width // 2 + jitter // 2

    # Draw bright track corridor polygon
    road_poly = [
        (top_center_x - 35, horizon_y),
        (top_center_x + 35, horizon_y),
        (bottom_center_x + 125, height),
        (bottom_center_x - 125, height),
    ]
    draw.polygon(road_poly, fill=(195, 205, 215))

    # Center lane guide line
    draw.line([(top_center_x, horizon_y), (bottom_center_x, height)], fill=(245, 190, 40), width=5)

    # If command is 'S' (Stop), draw a close red/orange obstacle box blocking the road
    if cmd == "S":
        obs_x = width // 2 + int(rng.integers(-20, 21))
        draw.rectangle([obs_x - 55, horizon_y + 25, obs_x + 55, height - 25], fill=(220, 50, 45))

    arr = np.array(img, dtype=np.uint8)
    noise = rng.integers(-10, 11, size=arr.shape, dtype=np.int16)
    return np.clip(arr.astype(np.int16) + noise, 0, 255).astype(np.uint8)


def run_simulation_collection(dataset_dir: str, total_samples: int) -> None:
    csv_path = ensure_dataset_dirs(dataset_dir)
    print(f"[SIMULATE] Generating {total_samples} labeled road frames in '{dataset_dir}/'...")
    counts = {c: 0 for c in VALID_CLASSES}

    for i in range(total_samples):
        cmd = VALID_CLASSES[i % len(VALID_CLASSES)]
        frame_rgb = generate_synthetic_road_frame(cmd)
        save_labeled_frame(frame_rgb, cmd, dataset_dir, csv_path)
        counts[cmd] += 1

    print(f"[SIMULATE COMPLETE] Saved {total_samples} frames across classes: {counts}")
    print(f"Driving log written to: {csv_path}")


def run_live_collection(esp_ip: str, dataset_dir: str) -> None:
    if not HAS_CV2:
        raise RuntimeError("OpenCV (cv2) is required for live interactive window. Install: pip install opencv-python")

    csv_path = ensure_dataset_dirs(dataset_dir)
    stream_url = f"http://{esp_ip}:81/stream"
    print("=================================================================")
    print(f" Connecting to ESP32-CAM stream: {stream_url}")
    print(" Controls: W=Forward | Q=CurveLeft | E=CurveRight | A=SpinLeft | D=SpinRight | S/Space=Stop")
    print(" Toggle Recording: Press 'R' | Quit: Press 'ESC'")
    print("=================================================================")

    cap = cv2.VideoCapture(stream_url)
    if not cap.isOpened():
        print("[ERROR] Could not open stream. Verify ESP32-CAM IP and Wi-Fi connection.")
        return

    recording = True
    current_cmd = "S"
    saved_count = 0
    last_save_time = 0.0

    # Put Arduino into Camera/ML mode ('C')
    send_car_command_async(esp_ip, "C")

    while True:
        ret, frame_bgr = cap.read()
        if not ret:
            time.sleep(0.02)
            continue

        key = cv2.waitKey(30) & 0xFF
        if key == 27:  # ESC
            send_car_command_async(esp_ip, "S")
            break
        elif key in (ord("r"), ord("R")):
            recording = not recording
            print(f"[RECORDING] {'ENABLED' if recording else 'PAUSED'}")
        elif key in KEY_TO_CMD:
            new_cmd = KEY_TO_CMD[key]
            if new_cmd != current_cmd:
                current_cmd = new_cmd
                send_car_command_async(esp_ip, current_cmd)

        # Save frame at ~8 Hz while actively driving
        now = time.time()
        if recording and current_cmd != "S" and (now - last_save_time >= 0.12):
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            save_labeled_frame(frame_rgb, current_cmd, dataset_dir, csv_path)
            saved_count += 1
            last_save_time = now

        # Draw HUD overlay
        hud = frame_bgr.copy()
        status_color = (0, 255, 0) if recording else (0, 165, 255)
        cv2.putText(
            hud,
            f"CMD: {current_cmd} | REC: {'ON' if recording else 'OFF'} | Saved: {saved_count}",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            status_color,
            2,
        )
        cv2.imshow("ESP32-CAM Data Collector", hud)

    cap.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Collect labeled driving frames from ESP32-CAM.")
    parser.add_argument("--ip", type=str, default="192.168.4.1", help="ESP32-CAM IP address")
    parser.add_argument("--dataset", type=str, default="dataset", help="Dataset output directory")
    parser.add_argument("--simulate", action="store_true", help="Generate synthetic dataset to test ML pipeline")
    parser.add_argument("--samples", type=int, default=180, help="Number of synthetic samples when --simulate is set")
    args = parser.parse_args()

    if args.simulate:
        run_simulation_collection(args.dataset, args.samples)
    else:
        run_live_collection(args.ip, args.dataset)


if __name__ == "__main__":
    main()
