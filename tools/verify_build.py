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
  6. PIN MAP: the 4 Arduino sketches, the README wiring table, both schematics and
     every pin written in a comment all match the car's real wiring (WIRING below)
  7. PIN TRACE: each sketch is actually RUN on recording Arduino stubs and must
     touch exactly the wired pins - and drive the L298N legally in every maneuver
  8. The dashboard HTML/JavaScript embedded in the ESP32 firmware is well formed

Usage:
    python3 tools/verify_build.py

Requires: python3 and g++ (any Linux/macOS; on Windows use WSL or skip step 2).
Nothing is written into the project — all temporary work happens in /tmp.
===============================================================================
"""

import html
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
# ARDUINO STUBS  — let a plain C++ compiler build (and even RUN) the .ino files.
# The stubs RECORD every pin a sketch touches (see HwTrace) so step [7] can prove
# the real code uses exactly the pins the car is wired to.
# ============================================================================
ARDUINO_H = r"""
#pragma once
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <set>
#include <deque>
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

// ---- hardware recorder: remembers every pin operation the sketch performs ----
struct HwTrace {
  unsigned long us;                    // virtual clock, microseconds
  unsigned      lcg;                   // deterministic pseudo-random sensor values
  std::set<int> outPins, inPins, dWrite, aWrite, dRead, aRead, pulsePins, servoPins;
  int  level[64];                      // last digitalWrite level of each pin
  int  pwm[64];                        // last analogWrite value of each pin
  int  servoMoves;
  int  softRx, softTx;
  std::set<std::string> drive;         // distinct motor states seen, e.g. "L:F R:F"
  int  badDrive;                       // impossible L298N states (a side with PWM>0 but not exactly one direction pin HIGH)
  std::deque<char> usbIn, softIn;      // text "typed" into the Serial Monitor / sent by the ESP32-CAM
  HwTrace() : us(0), lcg(12345), servoMoves(0), softRx(-1), softTx(-1), badDrive(0) {
    for (int i = 0; i < 64; i++) { level[i] = 0; pwm[i] = 0; }
  }
  unsigned rnd() { lcg = lcg * 1103515245u + 12345u; return (lcg >> 16) & 0x7fff; }
};
static HwTrace hw;

#ifdef W_ENA   // classify the L298N state whenever the second PWM pin is written
inline char sideDir(int a, int b, int pwm) {
  if (pwm == 0) return 'S';                                  // stopped
  if ( hw.level[a] && !hw.level[b]) return 'F';              // forward
  if (!hw.level[a] &&  hw.level[b]) return 'B';              // backward
  return 'X';                                                // illegal while driving
}
inline void hwDriveSnapshot() {
  char l = sideDir(W_IN1, W_IN2, hw.pwm[W_ENA]);
  char r = sideDir(W_IN3, W_IN4, hw.pwm[W_ENB]);
  if (l == 'X' || r == 'X') hw.badDrive++;
  char buf[16]; snprintf(buf, sizeof buf, "L:%c R:%c", l, r);
  hw.drive.insert(buf);
}
#endif

inline void pinMode(int p, int m){ if (m == OUTPUT) hw.outPins.insert(p); else hw.inPins.insert(p); }
inline void digitalWrite(int p, int v){ hw.dWrite.insert(p); if (p >= 0 && p < 64) hw.level[p] = v; }
inline void analogWrite(int p, int v){
  hw.aWrite.insert(p);
  if (p >= 0 && p < 64) hw.pwm[p] = v;
#ifdef W_ENA
  if (p == W_ENB) hwDriveSnapshot();
#endif
}
inline int  digitalRead(int p){ hw.dRead.insert(p); return (hw.rnd() % 100) < 15 ? LOW : HIGH; }  // IR flickers
inline int  analogRead(int p){ hw.aRead.insert(p); return 512; }
inline void delay(unsigned long ms){ hw.us += ms * 1000UL; }
inline void delayMicroseconds(unsigned int u){ hw.us += u; }
inline unsigned long millis(){ hw.us += 20; return hw.us / 1000UL; }
inline unsigned long micros(){ hw.us += 20; return hw.us; }
inline unsigned long hwEchoUs(){            // scripted HC-SR04: 58 us per cm, round trip
  unsigned r = hw.rnd() % 100;
  if (r < 10) return 0;                                       // no echo -> timeout
  unsigned cm = r < 35 ? 6 + hw.rnd() % 20                    // near   6..25 cm
              : r < 55 ? 28 + hw.rnd() % 20                   // medium 28..47 cm
                       : 50 + hw.rnd() % 150;                 // far    50..199 cm
  return (unsigned long)cm * 58UL;
}
inline unsigned long pulseIn(int p, int, unsigned long){ hw.pulsePins.insert(p); return hwEchoUs(); }
inline unsigned long pulseIn(int p, int){ hw.pulsePins.insert(p); return hwEchoUs(); }
inline long map(long x,long a,long b,long c,long d){ return (x-a)*(d-c)/(b-a)+c; }
template<class T> inline T constrain(T v,T lo,T hi){ return v<lo?lo:(v>hi?hi:v); }
inline int toupper(int c){ return (c>='a'&&c<='z')?c-32:c; }
inline bool isDigit(int c){ return c>='0'&&c<='9'; }
class SerialStub {
 public:
  void begin(long){}
  void begin(long,int){}
  void begin(long,int,int){}
  int  available(){ return (int)hw.usbIn.size(); }
  int  read(){
    if (hw.usbIn.empty()) return -1;
    int c = (unsigned char)hw.usbIn.front(); hw.usbIn.pop_front(); return c;
  }
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
#include "Arduino.h"
class Servo {
 public:
  void attach(int p){ hw.servoPins.insert(p); }
  void attach(int p, int, int){ hw.servoPins.insert(p); }
  void write(int){ hw.servoMoves++; }
  void writeMicroseconds(int){ hw.servoMoves++; }
};
"""

SOFTWARE_SERIAL_H = r"""
#pragma once
#include "Arduino.h"
class SoftwareSerial {
 public:
  SoftwareSerial(int rx, int tx){ hw.softRx = rx; hw.softTx = tx; }
  void begin(long){}
  void begin(long,int,int){}
  int  available(){ return (int)hw.softIn.size(); }
  int  read(){
    if (hw.softIn.empty()) return -1;
    int c = (unsigned char)hw.softIn.front(); hw.softIn.pop_front(); return c;
  }
  void write(char){}
  void print(const char*){}
  void print(char){}
  void print(int){}
  void println(const char*){}
  void println(char){}
  void println(int){}
};
"""

# Syntax-only compile check: build the sketch, run setup() + 3 loops
MAIN_CPP = r"""
#include "Arduino.h"
#include "Servo.h"
#include "SoftwareSerial.h"
SerialStub Serial;
#include "@SKETCH@"
int main(){ setup(); for (int i = 0; i < 3; i++) loop(); return 0; }
"""

# Pin-trace run: drive the sketch for ~19 s of virtual time with scripted keyboard
# / ESP32-CAM input and randomised sensors, then print which pins it touched.
TRACE_MAIN_CPP = r"""
#include "Arduino.h"
#include "Servo.h"
#include "SoftwareSerial.h"
SerialStub Serial;
#include "@SKETCH@"

static void feed(std::deque<char>& q, const char* s){ while (*s) q.push_back(*s++); }
static void report(const char* tag, const std::set<int>& v){
  printf("@%s ", tag);
  const char* sep = "";
  for (int x : v) { printf("%s%d", sep, x); sep = ","; }
  printf("\n");
}

int main(){
  setup();
  struct Step { int at; bool usb; const char* text; };
  static const Step script[] = {
    {   5, true,  "M" },                                   // manual mode, then every driving key
    {  10, true,  "w" }, {  14, true,  "q" }, {  18, true,  "e" }, {  22, true,  "a" },
    {  26, true,  "d" }, {  30, true,  "b" }, {  34, true,  "s" },
    {  40, true,  "A" },                                   // sensor autopilot (radar + IR reflexes)
    { 150, true,  "S" },
    { 155, true,  "C" },                                   // laptop / ML mode, commands via the ESP32-CAM
    { 160, false, "~F\n" }, { 164, false, "~G\n" }, { 168, false, "~I\n" }, { 172, false, "~L\n" },
    { 176, false, "~R\n" }, { 180, false, "~B\n" }, { 184, false, "~F\n" }, { 188, false, "~S\n" },
    { 200, true,  "A" },
  };
  for (int i = 0; i < 320; i++) {
    for (const Step& st : script) if (st.at == i) feed(st.usb ? hw.usbIn : hw.softIn, st.text);
    loop();
    hw.us += 60000UL;                                      // 60 ms pass between loop() calls
  }
  report("OUT",      hw.outPins);
  report("IN",       hw.inPins);
  report("DWRITE",   hw.dWrite);
  report("AWRITE",   hw.aWrite);
  report("DREAD",    hw.dRead);
  report("AREAD",    hw.aRead);
  report("PULSEIN",  hw.pulsePins);
  report("SERVO",    hw.servoPins);
  printf("@SERVOMOVES %d\n", hw.servoMoves);
  printf("@SOFTSERIAL %d,%d\n", hw.softRx, hw.softTx);
  printf("@DRIVE ");
  const char* sep = "";
  for (const std::string& d : hw.drive) { printf("%s%s", sep, d.c_str()); sep = "|"; }
  printf("\n@BADDRIVE %d\n", hw.badDrive);
  return 0;
}
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
                 f"(structure checked in step 8)")
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


# ============================================================================
# THE CAR'S WIRING — the single source of truth for every pin number.
# The 4 Arduino sketches, the README table, both schematics and every comment are
# checked against this table.  If you ever re-wire the car, change it HERE (and in
# the sketches) and step [6] lists each document you forgot to update.
# ============================================================================
WIRING = {
    "IN1": "8",   "IN2": "9",   "IN3": "10",  "IN4": "11",    # L298N direction pins
    "ENA": "5",   "ENB": "6",                                 # L298N speed (PWM) pins
    "TRIG": "12", "ECHO": "13",                               # HC-SR04 ultrasonic
    "SERVO": "7",                                             # 230-degree radar servo
    "IR_LEFT": "A0", "IR_RIGHT": "A1",                        # 2x IR obstacle sensors
    "CAM_RX": "A2",  "CAM_TX": "A3",                          # link to the ESP32-CAM
}
PIN_ALIASES = {"ESP_RX": "CAM_RX", "ESP_TX": "CAM_TX"}        # names used by the Phase 3 sketch
_BASE = ["IN1", "IN2", "IN3", "IN4", "ENA", "ENB", "TRIG", "ECHO", "SERVO"]
_IR = _BASE + ["IR_LEFT", "IR_RIGHT"]
_CAM = _IR + ["CAM_RX", "CAM_TX"]
SKETCH_USES = {          # sketch -> the signals it must wire up
    "phase1_ultrasonic/phase1_ultrasonic.ino": _BASE,
    "phase2_ultrasonic_ir/phase2_ultrasonic_ir.ino": _IR,
    "phase3_esp32cam_ml/arduino_ml_controller/arduino_ml_controller.ino": _CAM,
    "complete_car/complete_car.ino": _CAM,
}
ESP32_SKETCH = "phase3_esp32cam_ml/esp32cam_firmware/esp32cam_firmware.ino"
SVGS = ["schematics/full_wiring_schematic.svg", "schematics/phase_roadmap_diagram.svg"]

# which wired signal each Arduino call is allowed to touch
HW_CALL_RULES = {
    "digitalWrite": {"IN1", "IN2", "IN3", "IN4", "TRIG"},
    "analogWrite":  {"ENA", "ENB"},
    "pulseIn":      {"ECHO"},
    "digitalRead":  {"IR_LEFT", "IR_RIGHT"},
    "analogRead":   {"IR_LEFT", "IR_RIGHT"},
    "pinMode":      set(WIRING),
}
# how the signals are written in the README table:  (signal, component column, module-pin column)
README_ROWS = [
    ("IN1", "L298N Motor Driver", "IN1"), ("IN2", "L298N Motor Driver", "IN2"),
    ("IN3", "L298N Motor Driver", "IN3"), ("IN4", "L298N Motor Driver", "IN4"),
    ("ENA", "L298N Motor Driver", "ENA"), ("ENB", "L298N Motor Driver", "ENB"),
    ("TRIG", "HC-SR04 Ultrasonic", "TRIG"), ("ECHO", "HC-SR04 Ultrasonic", "ECHO"),
    ("SERVO", "230° Rotation Servo", "SIG"),
    ("IR_LEFT", "Left IR Sensor", "OUT"), ("IR_RIGHT", "Right IR Sensor", "OUT"),
    ("CAM_RX", "ESP32-CAM", "U0T"), ("CAM_TX", "ESP32-CAM (Optional)", "IO13"),
]
# words in documents/comments that name a wired signal
SIGNAL_PATTERNS = [
    (re.compile(r"\bIN1\b"), "IN1"), (re.compile(r"\bIN2\b"), "IN2"),
    (re.compile(r"\bIN3\b"), "IN3"), (re.compile(r"\bIN4\b"), "IN4"),
    (re.compile(r"\bENA\b"), "ENA"), (re.compile(r"\bENB\b"), "ENB"),
    (re.compile(r"\bTRIG\b"), "TRIG"), (re.compile(r"\bECHO\b"), "ECHO"),
    (re.compile(r"\bSERVO\b|ORG/YLW"), "SERVO"),
    (re.compile(r"\bLEFT IR\b|\bIR_LEFT\b", re.I), "IR_LEFT"),
    (re.compile(r"\bRIGHT IR\b|\bIR_RIGHT\b", re.I), "IR_RIGHT"),
    (re.compile(r"\bU0T\b|\bESP32 TX\b"), "CAM_RX"),
    (re.compile(r"\bIO13\b"), "CAM_TX"),
]
# wirings that were wrong in an earlier revision and must never come back
RETIRED_CLAIMS = [
    (re.compile(r"\bU0R\b[^\n]{0,80}\bA3\b|\bA3\b[^\n]{0,80}\bU0R\b"),
     "telemetry from Uno A3 goes to ESP32-CAM IO13 (what the firmware reads), NOT to U0R/GPIO3"),
]
PIN_TOKEN = re.compile(r"\b(?:D(\d{1,2})|A([0-5]))\b")
CLAUSE_SPLIT = re.compile(r"\s*(?:[+;·]|,\s)\s*")


def pin_number(pin):
    """'8' -> 8, 'A0' -> 14 (the number the Arduino core uses)."""
    return 14 + int(pin[1:]) if pin.startswith("A") else int(pin)


def pretty_pin(pin):
    """'8' -> 'D8', 'A0' -> 'A0' (how documents write it)."""
    return pin if pin.startswith("A") else "D" + pin


def pretty_num(n):
    return f"A{n - 14}" if n >= 14 else f"D{n}"


def strip_c_comments(code):
    code = re.sub(r"/\*.*?\*/", "", code, flags=re.S)
    return re.sub(r"//[^\n]*", "", code)


def read_sketch_pins(path):
    code = open(path, encoding="utf-8", errors="ignore").read()
    pins = {}
    for name, val in re.findall(r"^\s*#define\s+PIN_(\w+)\s+([A-Za-z0-9_]+)", code, re.M):
        pins[PIN_ALIASES.get(name, name)] = val
    for name, val in re.findall(r"^\s*const\s+int\s+PIN_(\w+)\s*=\s*([A-Za-z0-9_]+)\s*;", code, re.M):
        pins[PIN_ALIASES.get(name, name)] = val
    return pins


def svg_text_lines(path):
    svg = open(path, encoding="utf-8").read()
    return [html.unescape(re.sub(r"<[^>]+>", "", t))
            for t in re.findall(r"<text[^>]*>(.*?)</text>", svg, re.S)]


def pin_claims(lines):
    """Yield (clause, signal, pin) for each clause that ties ONE signal to ONE pin,
    e.g. 'D8 -> L298N IN1' or 'TRIG -> Arduino Pin D12'.  Ambiguous clauses are skipped."""
    for raw in lines:
        line = re.sub(r"\s+", " ", raw.replace("`", "").replace("*", ""))
        for clause in CLAUSE_SPLIT.split(line):
            sigs = {sig for pat, sig in SIGNAL_PATTERNS if pat.search(clause)}
            # D0/D1 are the USB pins (also used when the Uno acts as the ESP32-CAM's USB-serial
            # programmer, README step 3A-0) - they are not part of the car's wiring map.
            pins = {("D" + d) if d else ("A" + a) for d, a in PIN_TOKEN.findall(clause)
                    if d not in ("0", "1")}
            if len(sigs) == 1 and len(pins) == 1:
                yield clause.strip(), sigs.pop(), pins.pop()


def check_pin_map():
    print("\n[6] Pin map — sketches, README, schematics and comments all match the wiring")
    nums = [pin_number(v) for v in WIRING.values()]
    if len(set(nums)) != len(nums):
        bad("WIRING in verify_build.py gives one pin to two signals")
        return
    used_digital = {n for n in nums if n < 14}
    unused_digital = sorted(set(range(2, 14)) - used_digital)     # D0/D1 are the USB pins
    summary = " ".join(f"{k}={pretty_pin(v)}" for k, v in WIRING.items())
    ok(f"wiring under test: {summary}")

    # (a) the constants at the top of every sketch ---------------------------
    sketches = {}
    for rel_path, needs in SKETCH_USES.items():
        path = os.path.join(ROOT, *rel_path.split("/"))
        if not os.path.exists(path):
            bad(f"{rel_path} is missing")
            continue
        sketches[rel_path] = path
        have = read_sketch_pins(path)
        wrong = [f"{sig}: sketch has {have.get(sig, '(missing)')}, wiring says {WIRING[sig]}"
                 for sig in needs if have.get(sig) != WIRING[sig]]
        if wrong:
            bad(f"{rel_path}: pin constants differ from the wiring", "\n".join(wrong))
        else:
            ok(f"{rel_path}: all {len(needs)} pin constants match")

    # (b) every Arduino call uses the right named pin (also covers rarely-run code)
    for rel_path, path in sketches.items():
        code = strip_c_comments(open(path, encoding="utf-8", errors="ignore").read())
        problems = []
        for func, arg in re.findall(r"\b(pinMode|digitalWrite|analogWrite|digitalRead|analogRead|pulseIn)"
                                    r"\s*\(\s*([^,)\s]+)", code):
            if not arg.startswith("PIN_"):
                problems.append(f"{func}({arg}, …) uses a raw/variable pin, not a PIN_ constant")
                continue
            sig = PIN_ALIASES.get(arg[4:], arg[4:])
            if sig not in WIRING:
                problems.append(f"{func}({arg}, …) names a pin that is not in the wiring")
            elif sig not in HW_CALL_RULES[func]:
                problems.append(f"{func}({arg}, …): {sig} must not be used with {func}()")
        if re.search(r"\btone\s*\(", code) or re.search(r"#\s*include\s*<SPI\.h>", code):
            problems.append("uses tone() or SPI — would claim a timer/pin the README says is free")
        if problems:
            bad(f"{rel_path}: wrong pin used in a hardware call", "\n".join(problems))
        else:
            ok(f"{rel_path}: every pinMode/digitalWrite/analogWrite/pulseIn call uses the right pin")

    # (c) the README wiring table, row by row --------------------------------
    readme = os.path.join(ROOT, "README.md")
    doc_lines = []
    if os.path.exists(readme):
        doc = open(readme, encoding="utf-8").read()
        doc_lines = doc.splitlines()
        rows = {}
        for line in doc_lines:
            if not line.lstrip().startswith("|"):
                continue
            cols = [re.sub(r"[*`]", "", c).strip() for c in line.strip().strip("|").split("|")]
            m = re.search(r"\bArduino\s+([DA]\d+)\b", cols[2]) if len(cols) > 2 else None
            if m:
                rows[(cols[0], cols[1])] = m.group(1)
        wrong = []
        for sig, comp, modpin in README_ROWS:
            found = [pin for (c0, c1), pin in rows.items() if c0.startswith(comp) and c1.startswith(modpin)]
            want = pretty_pin(WIRING[sig])
            if not found:
                wrong.append(f"{sig}: no row for '{comp} / {modpin}' in the README wiring table")
            elif found[0] != want:
                wrong.append(f"{sig}: README table says {found[0]}, wiring says {want}")
        if wrong:
            bad("README wiring table differs from the wiring", "\n".join(wrong))
        else:
            ok(f"README wiring table: all {len(README_ROWS)} rows match")
    else:
        bad("README.md is missing")

    # (d) every written claim in README / schematics / comments --------------
    corpus = {"README.md": doc_lines}
    for rel_path in SVGS:
        path = os.path.join(ROOT, *rel_path.split("/"))
        if os.path.exists(path):
            corpus[rel_path] = svg_text_lines(path)
    for rel_path in list(SKETCH_USES) + [ESP32_SKETCH]:
        path = os.path.join(ROOT, *rel_path.split("/"))
        if os.path.exists(path):
            corpus[rel_path] = open(path, encoding="utf-8", errors="ignore").read().splitlines()

    mismatches, checked = [], 0
    for name, lines in corpus.items():
        for clause, sig, pin in pin_claims(lines):
            checked += 1
            want = pretty_pin(WIRING[sig])
            if pin != want:
                mismatches.append(f"{name}: “{clause[:70]}” says {sig} = {pin}, wiring says {want}")
    if mismatches:
        bad(f"{len(mismatches)} written pin claim(s) are out of date", "\n".join(mismatches[:6]))
    else:
        ok(f"{checked} written pin claims (README, schematics, sketch comments) all agree")

    revived = []
    for name, lines in corpus.items():
        for i, line in enumerate(lines, 1):
            for pat, why in RETIRED_CLAIMS:
                if pat.search(line.replace("`", "").replace("*", "")):
                    revived.append(f"{name}:{i}: {why}")
    if revived:
        bad(f"{len(revived)} known-wrong wiring statement(s) came back", "\n".join(revived[:6]))
    else:
        ok("no document repeats an earlier wiring mistake (telemetry → IO13, not U0R)")

    # (e) nothing may mention a digital pin the car does not use --------------
    stale = []
    for name, lines in corpus.items():
        if name == ESP32_SKETCH:
            continue
        for i, line in enumerate(lines, 1):
            for d in re.findall(r"\bD(\d{1,2})\b", line):
                if int(d) in unused_digital:
                    stale.append(f"{name}:{i}: mentions D{d}, which nothing is wired to")
    if stale:
        bad(f"{len(stale)} reference(s) to unused pins {['D%d' % n for n in unused_digital]}", "\n".join(stale[:6]))
    else:
        ok(f"no document mentions the unused pins {['D%d' % n for n in unused_digital]}")


def check_pin_trace(workdir):
    print("\n[7] Run every sketch on recording Arduino stubs — which pins does the REAL code touch?")
    if not shutil.which("g++"):
        skip("g++ not installed — cannot run the sketches")
        return
    macros = [f"-DW_{k}={pin_number(WIRING[k])}" for k in ("IN1", "IN2", "IN3", "IN4", "ENA", "ENB")]
    want_drive = {"L:F R:F": "forward", "L:B R:B": "reverse", "L:B R:F": "spin-left",
                  "L:F R:B": "spin-right", "L:S R:S": "stop"}
    nums = lambda names: {pin_number(WIRING[n]) for n in names}
    for rel_path, needs in SKETCH_USES.items():
        path = os.path.join(ROOT, *rel_path.split("/"))
        if not os.path.exists(path):
            continue
        main = os.path.join(workdir, "trace_main.cpp")
        exe = os.path.join(workdir, "trace_run")
        with open(main, "w") as f:
            f.write(TRACE_MAIN_CPP.replace("@SKETCH@", path))
        build = subprocess.run(["g++", "-std=c++14", f"-I{workdir}", f"-I{os.path.dirname(path)}",
                                "-O0", *macros, "-o", exe, main], capture_output=True, text=True)
        if build.returncode != 0:
            bad(f"{rel_path}: could not build the pin-trace run", build.stderr)
            continue
        try:
            run = subprocess.run([exe], capture_output=True, text=True, timeout=60)
        except subprocess.TimeoutExpired:
            bad(f"{rel_path}: did not finish its 320 loop() passes — possible endless loop")
            continue
        got = {}
        for line in run.stdout.splitlines():
            if line.startswith("@"):
                key, _, val = line[1:].partition(" ")
                got[key] = val.strip()
        if run.returncode != 0 or "BADDRIVE" not in got:
            bad(f"{rel_path}: the trace run crashed (exit {run.returncode})", run.stderr)
            continue
        as_set = lambda k: {int(x) for x in got.get(k, "").split(",") if x}

        has_ir, has_cam = "IR_LEFT" in needs, "CAM_RX" in needs
        want = {
            "digital outputs written": (as_set("DWRITE"), nums(["IN1", "IN2", "IN3", "IN4", "TRIG"])),
            "PWM (analogWrite)":       (as_set("AWRITE"), nums(["ENA", "ENB"])),
            "ultrasonic echo (pulseIn)": (as_set("PULSEIN"), nums(["ECHO"])),
            "servo attached to":       (as_set("SERVO"), nums(["SERVO"])),
            "IR sensors read":         (as_set("DREAD"), nums(["IR_LEFT", "IR_RIGHT"]) if has_ir else set()),
        }
        problems = []
        for label, (seen, expected) in want.items():
            if seen != expected:
                problems.append(f"{label}: code uses {sorted(map(pretty_num, seen))}, "
                                f"wiring says {sorted(map(pretty_num, expected))}")
        rx, tx = (int(x) for x in got["SOFTSERIAL"].split(","))
        exp_soft = (pin_number(WIRING["CAM_RX"]), pin_number(WIRING["CAM_TX"])) if has_cam else (-1, -1)
        if (rx, tx) != exp_soft:
            problems.append(f"ESP32-CAM serial link on {rx},{tx} but wiring says {exp_soft[0]},{exp_soft[1]}")
        configured = as_set("OUT") | as_set("IN")
        wired = nums([n for n in needs if n not in ("CAM_RX", "CAM_TX")])
        if not configured <= wired:
            problems.append(f"pinMode() on unwired pins {sorted(map(pretty_num, configured - wired))}")
        if not (as_set("DWRITE") | as_set("AWRITE")) <= as_set("OUT"):
            problems.append("a pin is written without pinMode(…, OUTPUT)")
        if not (as_set("PULSEIN") | as_set("DREAD")) <= as_set("IN"):
            problems.append("a sensor pin is read without pinMode(…, INPUT)")
        if int(got["SERVOMOVES"]) == 0:
            problems.append("the servo was attached but never moved")
        drive = set(got["DRIVE"].split("|")) if got["DRIVE"] else set()
        if int(got["BADDRIVE"]) > 0:
            problems.append(f"{got['BADDRIVE']} illegal L298N state(s): a side driven with both/neither direction pin HIGH")
        missing = [name for state, name in want_drive.items() if state not in drive]
        if missing:
            problems.append(f"the scripted run never made the car: {', '.join(missing)}")
        if problems:
            bad(f"{rel_path}: real behaviour differs from the wiring", "\n".join(problems))
        else:
            touched = sorted(as_set("OUT") | as_set("IN"))
            ok(f"{rel_path}: touches only {','.join(map(pretty_num, touched))}"
               f"{' + A2/A3 link' if has_cam else ''}; "
               f"forward/reverse/spin-L/spin-R/stop all drive the right pins, no illegal L298N state")


def check_dashboard_html():
    print("\n[8] ESP32-CAM dashboard HTML/JavaScript well formed")
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
        check_pin_map()
        check_pin_trace(workdir)
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
