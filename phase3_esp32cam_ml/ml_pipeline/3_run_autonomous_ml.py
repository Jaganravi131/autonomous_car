#!/usr/bin/env python3
"""
===============================================================================
PHASE 3 (STEP 3): REAL-TIME AUTONOMOUS ML PILOT FOR ESP32-CAM + ARDUINO UNO
===============================================================================
Pulls live frames from the ESP32-CAM stream (`http://<ESP_IP>:81/stream`),
runs real-time Machine Learning inference (and/or Computer Vision road analysis),
smooths predictions with an Exponential Moving Average (EMA) filter, and sends
driving commands (`F`, `G`, `I`, `L`, `R`, `S`) to `http://<ESP_IP>/cmd?c=<CMD>`.

Meanwhile, the Arduino Uno continuously monitors the HC-SR04 Ultrasonic sensor
and 2x IR sensors every 15ms as a hard real-time collision override!

Usage (Live Autonomous Driving):
    python3 3_run_autonomous_ml.py --ip 192.168.4.1 --mode hybrid

Usage (Offline Benchmark / Verification on Dataset):
    python3 3_run_autonomous_ml.py --test-on-dataset dataset --model models/autonomous_car_model.npz
===============================================================================
"""

import argparse
import glob
import os
import time

import numpy as np
import requests
from PIL import Image

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

CLASSES = ["F", "G", "I", "L", "R", "S"]
IMG_W, IMG_H = 48, 36


class AutonomousMLPilot:
    """Real-time Neural Network + Computer Vision Hybrid Pilot."""

    def __init__(self, model_path: str = "models/autonomous_car_model.npz", mode: str = "hybrid"):
        self.mode = mode
        self.weights = None
        self.ema_probs = np.ones(len(CLASSES), dtype=np.float32) / len(CLASSES)

        if os.path.exists(model_path):
            data = np.load(model_path)
            self.weights = {
                "W1": data["W1"],
                "b1": data["b1"],
                "W2": data["W2"],
                "b2": data["b2"],
            }
            print(f"[ML PILOT] Loaded trained neural network weights from '{model_path}'")
        elif mode != "vision":
            print(f"[WARNING] Model '{model_path}' not found; falling back to 'vision' mode.")
            self.mode = "vision"

    @staticmethod
    def preprocess(img_rgb: np.ndarray) -> np.ndarray:
        h, w, _ = img_rgb.shape
        roi = img_rgb[int(h * 0.35):, :, :]
        pil_img = Image.fromarray(roi).resize((IMG_W, IMG_H), Image.Resampling.BILINEAR)
        arr = np.asarray(pil_img, dtype=np.float32) / 255.0
        pooled = arr.reshape(1, 9, 4, 12, 4, 3).mean(axis=(2, 4))
        return pooled.reshape(1, -1)

    def predict_ml_probs(self, img_rgb: np.ndarray) -> np.ndarray:
        feat = self.preprocess(img_rgb)
        z1 = feat @ self.weights["W1"] + self.weights["b1"]
        h1 = np.maximum(0.0, z1)
        logits = (h1 @ self.weights["W2"] + self.weights["b2"])[0]
        logits -= np.max(logits)
        exp_s = np.exp(logits)
        return exp_s / np.sum(exp_s)

    @staticmethod
    def predict_vision_probs(img_rgb: np.ndarray) -> np.ndarray:
        """
        Zero-training Computer Vision heuristic:
        Analyzes road ROI brightness centroid and obstacle color in the center lane.
        """
        h, w, _ = img_rgb.shape
        roi = img_rgb[int(h * 0.35):, :, :].astype(np.float32)

        # Detect close red/dark obstacle in center region
        c_roi = roi[:, int(w * 0.3):int(w * 0.7), :]
        red_dominance = np.mean(c_roi[:, :, 0] - 0.5 * (c_roi[:, :, 1] + c_roi[:, :, 2]))
        probs = np.full(len(CLASSES), 0.03, dtype=np.float32)

        if red_dominance > 65.0:
            probs[CLASSES.index("S")] = 0.85
            return probs / np.sum(probs)

        # Compute horizontal centroid of bright track pixels in top half of ROI
        top_roi = roi[:int(roi.shape[0] * 0.5), :, :]
        gray = np.mean(top_roi, axis=2)
        mask = (gray > np.percentile(gray, 65)).astype(np.float32)
        x_coords = np.arange(w, dtype=np.float32)
        centroid_x = float(np.sum(mask * x_coords) / (np.sum(mask) + 1e-5))
        offset = centroid_x - (w / 2.0)

        if abs(offset) < 18:
            probs[CLASSES.index("F")] = 0.80
        elif -52 <= offset <= -18:
            probs[CLASSES.index("G")] = 0.80
        elif 18 <= offset <= 52:
            probs[CLASSES.index("I")] = 0.80
        elif offset < -52:
            probs[CLASSES.index("L")] = 0.80
        else:
            probs[CLASSES.index("R")] = 0.80

        return probs / np.sum(probs)

    def decide_command(self, img_rgb: np.ndarray, smooth: bool = True) -> tuple[str, float, np.ndarray]:
        if self.mode == "ml" and self.weights is not None:
            raw_probs = self.predict_ml_probs(img_rgb)
        elif self.mode == "vision" or self.weights is None:
            raw_probs = self.predict_vision_probs(img_rgb)
        else:
            # Hybrid: 75% Trained Neural Network + 25% Computer Vision prior
            ml_p = self.predict_ml_probs(img_rgb)
            cv_p = self.predict_vision_probs(img_rgb)
            raw_probs = 0.75 * ml_p + 0.25 * cv_p

        if smooth:
            self.ema_probs = 0.65 * raw_probs + 0.35 * self.ema_probs
            final_probs = self.ema_probs
        else:
            final_probs = raw_probs

        best_idx = int(np.argmax(final_probs))
        return CLASSES[best_idx], float(final_probs[best_idx]), final_probs


def run_dataset_benchmark(dataset_dir: str, model_path: str, mode: str) -> None:
    pilot = AutonomousMLPilot(model_path=model_path, mode=mode)
    correct = 0
    total = 0
    t0 = time.perf_counter()

    for cls in CLASSES:
        files = sorted(glob.glob(os.path.join(dataset_dir, cls, "*.jpg")))
        for fpath in files:
            img_rgb = np.asarray(Image.open(fpath).convert("RGB"))
            pred_cmd, conf, _ = pilot.decide_command(img_rgb, smooth=False)
            if pred_cmd == cls:
                correct += 1
            total += 1

    elapsed_ms = (time.perf_counter() - t0) * 1000.0
    fps = total / max(elapsed_ms / 1000.0, 1e-6)
    acc = (correct / max(total, 1)) * 100.0
    print("=================================================================")
    print(f" OFFLINE AUTONOMOUS ML BENCHMARK ({mode.upper()} MODE)")
    print(f" Evaluated {total} road frames in {elapsed_ms:.1f} ms ({fps:.1f} FPS)")
    print(f" Classification Accuracy: {correct}/{total} ({acc:.1f}%)")
    print("=================================================================")


def run_live_autopilot(esp_ip: str, model_path: str, mode: str) -> None:
    if not HAS_CV2:
        raise RuntimeError("OpenCV (cv2) is required for live video stream. Install: pip install opencv-python")

    pilot = AutonomousMLPilot(model_path=model_path, mode=mode)
    stream_url = f"http://{esp_ip}:81/stream"
    cmd_url = f"http://{esp_ip}/cmd"
    ml_url = f"http://{esp_ip}/ml"   # optional: mirrors predictions to the dashboard

    print(f"[AUTOPILOT] Connecting to ESP32-CAM at {stream_url} (Mode: {mode})...")
    cap = cv2.VideoCapture(stream_url)
    if not cap.isOpened():
        print("[ERROR] Could not connect to ESP32-CAM stream. Check Wi-Fi and IP address.")
        return

    session = requests.Session()
    try:
        session.get(cmd_url, params={"c": "C"}, timeout=0.4)
    except requests.RequestException:
        pass

    last_sent_cmd = None
    last_send_time = 0.0
    last_push_time = 0.0

    try:
        while True:
            ret, frame_bgr = cap.read()
            if not ret:
                time.sleep(0.02)
                continue

            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            cmd, conf, _ = pilot.decide_command(frame_rgb, smooth=True)

            now = time.time()
            # Send command immediately on change, or refresh every 250ms for watchdog
            if cmd != last_sent_cmd or (now - last_send_time >= 0.25):
                try:
                    session.get(cmd_url, params={"c": cmd}, timeout=0.25)
                    last_sent_cmd = cmd
                    last_send_time = now
                except requests.RequestException:
                    pass

            # --- Mirror the prediction onto the web dashboard ----------------
            # Purely for visualisation: lets you watch what the model is
            # predicting in the browser while the car drives. Never affects
            # driving, and a failure here is deliberately ignored.
            if now - last_push_time >= 0.20:
                last_push_time = now
                try:
                    session.get(
                        ml_url,
                        params={"p": cmd, "c": int(round(conf * 100))},
                        timeout=0.15,
                    )
                except requests.RequestException:
                    pass

            hud = frame_bgr.copy()
            cv2.putText(
                hud,
                f"MODE: {mode.upper()} | PRED: {cmd} ({conf*100:.0f}%)",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 128),
                2,
            )
            cv2.imshow("Autonomous Car — Live ML Autopilot", hud)
            if (cv2.waitKey(1) & 0xFF) == 27:  # ESC to stop
                break
    finally:
        try:
            session.get(cmd_url, params={"c": "S"}, timeout=0.4)
        except requests.RequestException:
            pass
        cap.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Run real-time ML Autonomous Pilot for 4WD Car.")
    parser.add_argument("--ip", type=str, default="192.168.4.1", help="ESP32-CAM IP address")
    parser.add_argument("--model", type=str, default="models/autonomous_car_model.npz", help="Trained model path")
    parser.add_argument("--mode", choices=["ml", "vision", "hybrid"], default="hybrid", help="Inference mode")
    parser.add_argument("--test-on-dataset", type=str, default="", help="Run offline benchmark on dataset folder")
    args = parser.parse_args()

    if args.test_on_dataset:
        run_dataset_benchmark(args.test_on_dataset, args.model, args.mode)
    else:
        run_live_autopilot(args.ip, args.model, args.mode)


if __name__ == "__main__":
    main()
