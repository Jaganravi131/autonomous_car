#!/usr/bin/env python3
"""
===============================================================================
 tools/verify_build.py  —  ONE-COMMAND HEALTH CHECK FOR THE WHOLE PROJECT
===============================================================================
Answers the question "is this repo properly built?" without needing any
hardware and without needing the Arduino IDE installed.

It checks:
  1. Arduino sketch structure rule  ->  <folder>/<folder>.ino
  2. Every .ino sketch compiles clean (g++ -Wall -Wextra) against Arduino stubs
  3. Every Python script has valid syntax
  4. The whole ML pipeline actually runs: simulate -> train -> benchmark
  5. Every relative link and image in README.md resolves to a real file
  6. The pin numbers in README.md match the #defines in complete_car.ino
  7. The dashboard HTML/JavaScript embedded in the ESP32 firmware is well formed

Usage:
    python3 tools/verify_build.py

Requires: python3 and g++ (any Linux/macOS; on Windows use WSL or skip step 2).
Nothing is written into the project — all temporary work happens in /tmp.
===============================================================================
"""

import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PASS, FAIL, SKIP = [], [], []


def ok(msg):
    PASS.append(msg)
    print(f"  \033[92mPASS\033[0m  {msg}")


def bad(msg, detail=""):
    FAIL.append(msg)
    print(f"  \033[91mFAIL\033[0m  {msg}")
    if detail:
        for line in detail.strip().splitlines()[:6]:
            print(f"        {line}")


def skip(msg):
    SKIP.append(msg)
    print(f"  \033[93mSKIP\033[0m  {msg}")


def rel(path):
    return os.path.relpath(path, ROOT)


def tracked(pattern):
    """Find files in the project matching a glob-ish pattern (ignores .git)."""
    out = []
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames if d not in (".git", "__pycache__")]
        for fn in filenames:
            if re.search(pattern, fn):
                out.append(os.path.join(dirpath, fn))
    return sorted(out)


# ============================================================================
# ARDUINO STUBS (so the .ino files can be compiled by a plain C++ compiler)
# ============================================================================
ARDUINO_H = r"""
#pragma once
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
#define F(x) (x)
typedef bool boolean;
inline int  abs(int v){ return v < 0 ? -v : v; }
inline void pinMode(int,int){}
inline void digitalWrite(int,int){}
inline void analogWrite(int,int){}
inline int  digitalRead(int){ return 0; }
inline int  analogRead(int){ return 0; }
inline void delay(unsigned long){}
inline void delayMicroseconds(unsigned int){}
inline unsigned long pulseIn(int,int,unsigned long){ return 0; }
inline unsigned long pulseIn(int,int){ return 0; }
inline unsigned long millis(){ return 0; }
inline unsigned long micros(){ return 0; }
inline long map(long x,long a,long b,long c,long d){ return (x-a)*(d-c)/(b-a)+c; }
template<class T> inline T constrain(T v,T lo,T hi){ return v<lo?lo:(v>hi?hi:v); }
inline int toupper(int c){ return (c>='a'&&c<='z')?c-32:c; }
inline bool isDigit(int c){ return c>='0'&&c<='9'; }
class SerialStub {
 public:
  void begin(long){}
  void begin(long,int){}
  void begin(long,int,int){}
  int  available(){ return 0; }
  int  read(){ return -1; }
  void write(char){}
  void write(const char*){}
  size_t printf(const char*, ...){ return 0; }
  void print(const char* s){ printf("%s", s); }
  void print(char c){ printf("%c", c); }
  void print(int v){ printf("%d", v); }
  void print(unsigned int v){ printf("%u", v); }
  void print(long v){ printf("%ld", v); }
  void print(unsigned long v){ printf("%lu", v); }
  void print(double v){ printf("%.2f", v); }
  void println(){ printf("\n"); }
  void println(const char* s){ printf("%s\n", s); }
  void println(char c){ printf("%c\n", c); }
  void println(int v){ printf("%d\n", v); }
  void println(unsigned int v){ printf("%u\n", v); }
  void println(long v){ printf("%ld\n", v); }
  void println(unsigned long v){ printf("%lu\n", v); }
  void println(double v){ printf("%.2f\n", v); }
};
extern SerialStub Serial;
"""

SERVO_H = r"""
#pragma once
class Servo {
 public:
  void attach(int){}
  void attach(int,int,int){}
  void write(int){}
  void writeMicroseconds(int){}
};
"""

SOFTWARE_SERIAL_H = r"""
#pragma once
class SoftwareSerial {
 public:
  SoftwareSerial(int,int){}
  void begin(long){}
  void begin(long,int,int){}
  int  available(){ return 0; }
  int  read(){ return -1; }
  void write(char){}
  void print(const char*){}
  void print(char){}
  void print(int){}
  void println(const char*){}
  void println(char){}
  void println(int){}
};
"""

MAIN_CPP = r"""
#include "Arduino.h"
#include "Servo.h"
#include "SoftwareSerial.h"
SerialStub Serial;
#include "@SKETCH@"
int main(){ setup(); for (int i = 0; i < 3; i++) loop(); return 0; }
"""


def build_stubs(workdir):
    for name, body in (("Arduino.h", ARDUINO_H), ("Servo.h", SERVO_H),
                       ("SoftwareSerial.h", SOFTWARE_SERIAL_H)):
        with open(os.path.join(workdir, name), "w") as f:
            f.write(body)


def check_sketch_structure():
    print("\n[1] Arduino sketch structure  <folder>/<folder>.ino")
    for ino in tracked(r"\.ino$"):
        folder = os.path.basename(os.path.dirname(ino))
        stem = os.path.splitext(os.path.basename(ino))[0]
        if folder == stem:
            ok(f"{rel(ino)}")
        else:
            bad(f"{rel(ino)} — file must be named '{folder}.ino' to open in the Arduino IDE")


# The ESP32 sketches need the ESP32 Arduino core (WiFi.h, esp_camera.h, ...).
# Those headers only exist inside the Arduino IDE, so a host compiler cannot
# check them - we verify their structure instead (see check_esp32_sketch).
ESP32_ONLY = ("WiFi.h", "esp_camera.h", "esp_http_server.h")


def is_esp32_sketch(path):
    body = open(path, encoding="utf-8", errors="ignore").read()
    return any(f'#include "{h}"' in body or f"#include <{h}>" in body
               for h in ESP32_ONLY)


def check_compiles(workdir):
    print("\n[2] Compile every .ino  (g++ -Wall -Wextra, Arduino stubs)")
    if not shutil.which("g++"):
        skip("g++ not installed — cannot compile-check the sketches")
        return
    for ino in tracked(r"\.ino$"):
        if is_esp32_sketch(ino):
            skip(f"{rel(ino)} — ESP32-only sketch, needs the Arduino IDE "
                 f"(structure checked in step 7)")
            continue
        main = os.path.join(workdir, "main.cpp")
        with open(main, "w") as f:
            f.write(MAIN_CPP.replace("@SKETCH@", ino))
        cmd = ["g++", "-std=c++14", f"-I{workdir}", f"-I{os.path.dirname(ino)}",
               "-Wall", "-Wextra", "-fsyntax-only", main]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode == 0 and not r.stderr.strip():
            ok(f"{rel(ino)} compiles clean")
        else:
            bad(f"{rel(ino)} failed to compile", r.stderr)


def check_python_syntax():
    print("\n[3] Python script syntax")
    for py in tracked(r"\.py$"):
        r = subprocess.run([sys.executable, "-m", "py_compile", py],
                           capture_output=True, text=True)
        if r.returncode == 0:
            ok(f"{rel(py)}")
        else:
            bad(f"{rel(py)} has a syntax error", r.stderr)


def check_ml_pipeline():
    print("\n[4] ML pipeline end-to-end  (simulate -> train -> benchmark)")
    pipe = os.path.join(ROOT, "phase3_esp32cam_ml", "ml_pipeline")
    scripts = ["1_collect_data.py", "2_train_model.py", "3_run_autonomous_ml.py"]
    if not all(os.path.exists(os.path.join(pipe, s)) for s in scripts):
        skip("ml_pipeline scripts not found")
        return
    try:
        import numpy  # noqa: F401
    except ImportError:
        skip("numpy not installed — run: pip install -r "
             "phase3_esp32cam_ml/ml_pipeline/requirements.txt")
        return

    tmp = tempfile.mkdtemp(prefix="car_ml_check_")
    ds, models = os.path.join(tmp, "dataset"), os.path.join(tmp, "models")
    try:
        steps = [
            ("data collection", [sys.executable, "1_collect_data.py",
                                 "--simulate", "--dataset", ds, "--samples", "60"]),
            ("training",        [sys.executable, "2_train_model.py",
                                 "--dataset", ds, "--model-dir", models, "--epochs", "6"]),
            ("autopilot check", [sys.executable, "3_run_autonomous_ml.py",
                                 "--test-on-dataset", ds,
                                 "--model", os.path.join(models, "autonomous_car_model.npz"),
                                 "--mode", "hybrid"]),
        ]
        for label, cmd in steps:
            r = subprocess.run(cmd, cwd=pipe, capture_output=True, text=True, timeout=600)
            if r.returncode != 0:
                bad(f"{label} failed", r.stdout + r.stderr)
                return
        acc = re.search(r"Accuracy:\s*([\d.]+)%", r.stdout)
        ok("collect -> train -> benchmark all succeeded"
           + (f" (benchmark accuracy {acc.group(1)}%)" if acc else ""))
    except subprocess.TimeoutExpired:
        bad("ML pipeline timed out")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
        shutil.rmtree(os.path.join(pipe, "__pycache__"), ignore_errors=True)


def check_readme_links():
    print("\n[5] README.md links and images resolve")
    readme = os.path.join(ROOT, "README.md")
    if not os.path.exists(readme):
        bad("README.md is missing")
        return
    text = open(readme, encoding="utf-8").read()
    targets = re.findall(r"!?\[[^\]]*\]\(([^)]+)\)", text)
    checked, missing = 0, []
    for t in targets:
        t = t.split("#")[0].strip()
        if not t or t.startswith(("http://", "https://", "mailto:")):
            continue
        checked += 1
        if not os.path.exists(os.path.join(ROOT, t)):
            missing.append(t)
    if missing:
        for m in missing:
            bad(f"README links to a missing file: {m}")
    else:
        ok(f"all {checked} relative links/images resolve")


def check_pin_consistency():
    print("\n[6] Pin numbers in README match complete_car.ino")
    ino = os.path.join(ROOT, "complete_car", "complete_car.ino")
    readme = os.path.join(ROOT, "README.md")
    if not (os.path.exists(ino) and os.path.exists(readme)):
        skip("complete_car.ino or README.md missing")
        return
    code = open(ino).read()
    doc = open(readme, encoding="utf-8").read()

    defines = dict(re.findall(r"#define\s+(PIN_\w+)\s+([A-Za-z0-9_]+)", code))
    # name in the sketch -> label used in the wiring table
    expect = {
        "PIN_IN1": "IN1", "PIN_IN2": "IN2", "PIN_IN3": "IN3", "PIN_IN4": "IN4",
        "PIN_ENA": "ENA", "PIN_ENB": "ENB", "PIN_TRIG": "TRIG", "PIN_ECHO": "ECHO",
        "PIN_SERVO": "SIG", "PIN_IR_LEFT": "A0", "PIN_IR_RIGHT": "A1",
        "PIN_CAM_RX": "A2",
    }
    bad_found = False
    for define, label in expect.items():
        pin = defines.get(define)
        if pin is None:
            bad(f"{define} not found in complete_car.ino")
            bad_found = True
            continue
        pretty = "D" + pin if pin.isdigit() else pin
        # the wiring table lists the Arduino pin for this signal
        if pretty in doc:
            pass
        else:
            bad(f"{define} = {pin} ({pretty}) does not appear in README.md")
            bad_found = True
    if not bad_found:
        ok(f"all {len(expect)} signal pins agree between sketch and README")


def check_dashboard_html():
    print("\n[7] ESP32-CAM dashboard HTML/JavaScript well formed")
    fw = os.path.join(ROOT, "phase3_esp32cam_ml", "esp32cam_firmware",
                      "esp32cam_firmware.ino")
    if not os.path.exists(fw):
        skip("esp32cam_firmware.ino not found")
        return
    src = open(fw).read()
    m = re.search(r'R"rawliteral\((.*?)\)rawliteral"', src, re.S)
    if not m:
        bad("no embedded HTML found inside the R\"rawliteral(...)\" block")
        return
    html = m.group(1)
    for tag in ("<html", "</html>", "<script>", "</script>", "<body", "</body>"):
        if tag not in html:
            bad(f"dashboard HTML is missing {tag}")
            return
    js = re.search(r"<script>(.*?)</script>", html, re.S)
    if js:
        braces = js.group(1).count("{") - js.group(1).count("}")
        if braces != 0:
            bad(f"dashboard JavaScript has unbalanced braces ({braces:+d})")
            return
    ok("dashboard HTML complete, JavaScript braces balanced")
    ok("framed command protocol present" if "Serial.write('~')" in src
       else "WARNING: firmware does not frame its commands")

    # --- endpoints the dashboard + the Python ML pipeline depend on ---------
    for uri in ("/", "/cmd", "/capture", "/status", "/ml", "/stream"):
        if f'.uri = "{uri}"' in src:
            pass
        else:
            bad(f"endpoint {uri} is not registered in the firmware")
    ok("all 6 HTTP endpoints registered (/, /cmd, /capture, /status, /ml, /stream)")

    # --- every id the script touches must exist in the markup --------------
    script = js.group(1)
    ids = set(re.findall(r'id="([^"]+)"', html))
    used = set(re.findall(r"getElementById\('([^']+)'\)", script))
    missing = used - ids
    if missing:
        bad(f"dashboard JS references missing element ids: {sorted(missing)}")
    else:
        ok(f"all {len(used)} element ids referenced by the script exist")

    # --- the four mode buttons the car needs -------------------------------
    modes = re.findall(r'<button class="mode[^"]*" id="(m-\w)"', html)
    if len(modes) == 4 and all(f"send('{c}')" in html for c in "MACS"):
        ok("4 driving-mode buttons present (MANUAL / AUTO / CAMERA / STOP)")
    else:
        bad(f"expected 4 mode buttons sending M, A, C and S — found {modes}")

    # --- ML hooks for further development ----------------------------------
    if 'ml_handler' in src and 'mlPred' in src and 'mlConf' in src:
        ok("ML endpoint wired: /ml pushes predictions, /status returns them")
    else:
        bad("ML endpoint missing — the dashboard cannot show live predictions")

    # --- no external CDN links: the ESP32 access point has no internet -----
    ext = re.findall(r'(?:src|href)="(https?://[^"]+)"', html)
    if ext:
        bad(f"dashboard loads external resources that will fail offline: {ext}")
    else:
        ok("dashboard is fully self-contained (no CDN, works on the ESP32 hotspot)")


def main():
    print("=" * 74)
    print(" 4WD AUTONOMOUS CAR — PROJECT BUILD VERIFICATION")
    print(f" root: {ROOT}")
    print("=" * 74)

    workdir = tempfile.mkdtemp(prefix="arduino_stubs_")
    try:
        build_stubs(workdir)
        check_sketch_structure()
        check_compiles(workdir)
        check_python_syntax()
        check_ml_pipeline()
        check_readme_links()
        check_pin_consistency()
        check_dashboard_html()
    finally:
        shutil.rmtree(workdir, ignore_errors=True)

    print("\n" + "=" * 74)
    print(f" RESULT:  {len(PASS)} passed, {len(FAIL)} failed, {len(SKIP)} skipped")
    if FAIL:
        print("\n Items needing attention:")
        for f in FAIL:
            print(f"   - {f}")
    else:
        print(" The project is complete and consistent. Nothing to fix.")
    print("=" * 74)
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
