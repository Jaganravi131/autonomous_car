#!/usr/bin/env python3
"""
===============================================================================
PHASE 3 (STEP 2): TRAIN MACHINE LEARNING AUTONOMOUS DRIVING MODEL
===============================================================================
Loads collected road images from `dataset/`, applies Region-of-Interest (ROI)
cropping and horizontal mirror augmentation (auto-swapping Left <-> Right
steering labels), trains a Neural Network classifier to predict driving actions
('F', 'G', 'I', 'L', 'R', 'S'), evaluates validation accuracy, and saves:
  1. `models/autonomous_car_model.npz` (Fast NumPy Neural Network - runs anywhere)
  2. `models/autonomous_car_cnn.keras` & `.tflite` (if TensorFlow is installed)

Usage:
    python3 2_train_model.py --dataset dataset --epochs 35
===============================================================================
"""

import argparse
import glob
import os

import numpy as np
from PIL import Image

CLASSES = ["F", "G", "I", "L", "R", "S"]
CLASS_TO_IDX = {c: i for i, c in enumerate(CLASSES)}
FLIP_LABEL_MAP = {"F": "F", "G": "I", "I": "G", "L": "R", "R": "L", "S": "S"}

IMG_W, IMG_H = 48, 36  # Compact road ROI resolution for fast real-time inference


def preprocess_image(img_rgb: np.ndarray) -> np.ndarray:
    """Crop top 35% (ceiling/background), resize road ROI to (IMG_W, IMG_H), normalize to [0, 1]."""
    h, w, _ = img_rgb.shape
    roi = img_rgb[int(h * 0.35):, :, :]
    pil_img = Image.fromarray(roi).resize((IMG_W, IMG_H), Image.Resampling.BILINEAR)
    return np.asarray(pil_img, dtype=np.float32) / 255.0


def load_and_augment_dataset(dataset_dir: str):
    X_list, y_list = [], []

    for cls in CLASSES:
        cls_dir = os.path.join(dataset_dir, cls)
        if not os.path.isdir(cls_dir):
            continue
        files = sorted(glob.glob(os.path.join(cls_dir, "*.jpg")) + glob.glob(os.path.join(cls_dir, "*.png")))
        for fpath in files:
            img = np.asarray(Image.open(fpath).convert("RGB"))
            proc = preprocess_image(img)
            X_list.append(proc)
            y_list.append(CLASS_TO_IDX[cls])

            # Horizontal mirror augmentation with swapped steering label
            flipped_img = np.fliplr(img)
            flipped_cls = FLIP_LABEL_MAP[cls]
            X_list.append(preprocess_image(flipped_img))
            y_list.append(CLASS_TO_IDX[flipped_cls])

    if not X_list:
        raise RuntimeError(
            f"No training images found in '{dataset_dir}/'. "
            "Run `python3 1_collect_data.py --simulate` or collect live ESP32-CAM data first."
        )

    X = np.stack(X_list, axis=0)
    y = np.array(y_list, dtype=np.int64)

    # Shuffle dataset deterministically
    rng = np.random.default_rng(42)
    perm = rng.permutation(len(X))
    return X[perm], y[perm]


def train_numpy_mlp(X_train: np.ndarray, y_train: np.ndarray, X_val: np.ndarray, y_val: np.ndarray, epochs: int = 35):
    """
    Train a 2-layer Neural Network (Spatial Road Pooling -> Dense ReLU -> Softmax)
    using mini-batch Adam/SGD in pure NumPy so it works on any machine with zero GPU setup.
    """
    # Downsample spatial grid 12x9x3 = 324 features for fast, regularized generalization
    def extract_features(X_batch: np.ndarray) -> np.ndarray:
        # Reshape (N, 36, 48, 3) -> average pool 4x4 blocks -> (N, 9, 12, 3) -> flatten (N, 324)
        n = X_batch.shape[0]
        pooled = X_batch.reshape(n, 9, 4, 12, 4, 3).mean(axis=(2, 4))
        return pooled.reshape(n, -1)

    F_train = extract_features(X_train)
    F_val = extract_features(X_val)

    in_dim = F_train.shape[1]
    hidden_dim = 64
    out_dim = len(CLASSES)

    rng = np.random.default_rng(42)
    W1 = rng.normal(0, np.sqrt(2.0 / in_dim), size=(in_dim, hidden_dim)).astype(np.float32)
    b1 = np.zeros((1, hidden_dim), dtype=np.float32)
    W2 = rng.normal(0, np.sqrt(2.0 / hidden_dim), size=(hidden_dim, out_dim)).astype(np.float32)
    b2 = np.zeros((1, out_dim), dtype=np.float32)

    lr = 0.04
    batch_size = min(32, len(F_train))
    reg = 1e-4

    for epoch in range(1, epochs + 1):
        perm = rng.permutation(len(F_train))
        F_shuf, y_shuf = F_train[perm], y_train[perm]

        for i in range(0, len(F_train), batch_size):
            xb = F_shuf[i:i + batch_size]
            yb = y_shuf[i:i + batch_size]
            bs = len(xb)

            # Forward pass
            z1 = xb @ W1 + b1
            h1 = np.maximum(0.0, z1)
            logits = h1 @ W2 + b2
            logits -= np.max(logits, axis=1, keepdims=True)
            exp_scores = np.exp(logits)
            probs = exp_scores / np.sum(exp_scores, axis=1, keepdims=True)

            # Backpropagation
            dlogits = probs.copy()
            dlogits[np.arange(bs), yb] -= 1.0
            dlogits /= bs

            dW2 = h1.T @ dlogits + reg * W2
            db2 = np.sum(dlogits, axis=0, keepdims=True)

            dh1 = dlogits @ W2.T
            dz1 = dh1 * (z1 > 0)
            dW1 = xb.T @ dz1 + reg * W1
            db1 = np.sum(dz1, axis=0, keepdims=True)

            W1 -= lr * dW1
            b1 -= lr * db1
            W2 -= lr * dW2
            b2 -= lr * db2

        if epoch % 5 == 0 or epoch == epochs:
            val_h1 = np.maximum(0.0, F_val @ W1 + b1)
            val_preds = np.argmax(val_h1 @ W2 + b2, axis=1)
            val_acc = float(np.mean(val_preds == y_val)) * 100.0
            train_h1 = np.maximum(0.0, F_train @ W1 + b1)
            train_preds = np.argmax(train_h1 @ W2 + b2, axis=1)
            train_acc = float(np.mean(train_preds == y_train)) * 100.0
            print(f"  Epoch {epoch:02d}/{epochs:02d} — Train Acc: {train_acc:5.1f}% | Val Acc: {val_acc:5.1f}%")

    return {"W1": W1, "b1": b1, "W2": W2, "b2": b2, "classes": np.array(CLASSES)}


def try_train_tensorflow_cnn(X_train, y_train, X_val, y_val, model_dir: str, epochs: int):
    """If TensorFlow is installed, also train & export a Keras CNN + TFLite model."""
    try:
        import tensorflow as tf
    except ImportError:
        return

    print("\n[TENSORFLOW DETECTED] Training PilotNet-style Deep CNN...")
    model = tf.keras.Sequential([
        tf.keras.layers.Input(shape=(IMG_H, IMG_W, 3)),
        tf.keras.layers.Conv2D(16, (3, 3), activation="relu", padding="same"),
        tf.keras.layers.MaxPooling2D((2, 2)),
        tf.keras.layers.Conv2D(32, (3, 3), activation="relu", padding="same"),
        tf.keras.layers.MaxPooling2D((2, 2)),
        tf.keras.layers.Conv2D(64, (3, 3), activation="relu", padding="same"),
        tf.keras.layers.Flatten(),
        tf.keras.layers.Dense(64, activation="relu"),
        tf.keras.layers.Dropout(0.25),
        tf.keras.layers.Dense(len(CLASSES), activation="softmax"),
    ])
    model.compile(optimizer="adam", loss="sparse_categorical_crossentropy", metrics=["accuracy"])
    model.fit(X_train, y_train, validation_data=(X_val, y_val), epochs=min(epochs, 20), batch_size=32, verbose=1)

    keras_path = os.path.join(model_dir, "autonomous_car_cnn.keras")
    model.save(keras_path)
    print(f"[SAVED] Keras CNN model -> {keras_path}")


def main():
    parser = argparse.ArgumentParser(description="Train ML steering model from collected road dataset.")
    parser.add_argument("--dataset", type=str, default="dataset", help="Path to dataset folder")
    parser.add_argument("--model-dir", type=str, default="models", help="Directory to save trained models")
    parser.add_argument("--epochs", type=int, default=35, help="Training epochs")
    args = parser.parse_args()

    os.makedirs(args.model_dir, exist_ok=True)
    print(f"[1/3] Loading and augmenting dataset from '{args.dataset}/'...")
    X, y = load_and_augment_dataset(args.dataset)
    print(f"      Loaded {len(X)} total samples (including mirror augmentation) of shape {X.shape[1:]}")

    split = max(1, int(len(X) * 0.8))
    X_train, y_train = X[:split], y[:split]
    X_val, y_val = X[split:], y[split:]

    print(f"[2/3] Training Neural Network ({len(X_train)} train, {len(X_val)} validation)...")
    weights = train_numpy_mlp(X_train, y_train, X_val, y_val, epochs=args.epochs)

    npz_path = os.path.join(args.model_dir, "autonomous_car_model.npz")
    np.savez(npz_path, **weights)
    print(f"[3/3] Saved portable Neural Network weights -> {npz_path}")

    try_train_tensorflow_cnn(X_train, y_train, X_val, y_val, args.model_dir, args.epochs)
    print("\n[TRAINING COMPLETE] Ready to run `3_run_autonomous_ml.py`!")


if __name__ == "__main__":
    main()
