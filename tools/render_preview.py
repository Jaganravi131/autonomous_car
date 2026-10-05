#!/usr/bin/env python3
"""
Draws a static PREVIEW of the ESP32-CAM dashboard.

Why this exists: the ESP32 serves the dashboard from its own access point with
no internet, so the page cannot be rendered by a headless browser here. This
script reads the real HTML out of the firmware and draws the same layout with
PIL, so the picture can never drift far from the code.

It is a preview, NOT a screenshot. Regenerate after editing the firmware:

    python3 tools/render_preview.py
"""
import os
import re

from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FW = os.path.join(ROOT, "phase3_esp32cam_ml", "esp32cam_firmware", "esp32cam_firmware.ino")
OUT = os.path.join(ROOT, "dashboard_preview.png")

# ---- palette (kept identical to the CSS in the firmware) --------------------
BG = (11, 16, 32)
CARD = (23, 30, 48)
CARD_EDGE = (255, 255, 255, 26)
TEXT = (230, 237, 247)
MUTED = (139, 155, 180)
DIM = (100, 116, 139)
CYAN = (34, 211, 238)
GREEN = (34, 197, 94)
GREEN_L = (74, 222, 128)
RED = (239, 68, 68)
AMBER = (245, 158, 11)
VIOLET = (139, 92, 246)
PANEL = (15, 23, 42)


def font(size, bold=False):
    name = "DejaVuSans-Bold" if bold else "DejaVuSans"
    for p in (f"/usr/share/fonts/truetype/dejavu/{name}.ttf",
              f"/usr/share/fonts/truetype/dejavu/{name}.ttf"):
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


F_TITLE, F_NAME, F_SM, F_XS, F_BIG = font(15, 1), font(12, 1), font(11), font(10), font(21, 1)


def gradient(size, top, bottom):
    w, h = size
    img = Image.new("RGB", size)
    d = ImageDraw.Draw(img)
    for y in range(h):
        k = y / max(1, h - 1)
        d.line([(0, y), (w, y)], fill=tuple(int(top[i] + (bottom[i] - top[i]) * k) for i in range(3)))
    return img


def main():
    html = re.search(r'R"rawliteral\((.*?)\)rawliteral"', open(FW).read(), re.S).group(1)
    pads = re.findall(r'data-cmd="([^"]+)"[^>]*>.*?<span class="ar">(.*?)</span>'
                      r'<span class="lb">(.*?)</span>', html, re.S)
    modes = re.findall(r'<button class="mode[^"]*" id="(m-\w)"[^>]*>.*?class="ic">(.*?)</span>'
                       r'.*?class="nm">(.*?)</span>.*?class="ds">(.*?)</span>', html, re.S)
    print(f"read from firmware: {len(pads)} pad buttons, {len(modes)} mode buttons")

    W, PAD = 620, 30
    AW = W - 2 * PAD                      # app width
    y = 26
    canvas_h = 1500
    img = Image.new("RGB", (W, canvas_h), BG)
    # soft radial-ish tint at the top
    d = ImageDraw.Draw(img)
    for i in range(160):
        a = 1 - i / 160
        d.rectangle([0, i, W, i + 1], fill=(int(29 + 20 * a), int(43 + 20 * a), int(100 + 30 * a)))

    def card(h, label=""):
        nonlocal y
        overlay = Image.new("RGBA", (AW, h), (255, 255, 255, 13))
        img.paste(overlay, (PAD, y), overlay)
        d.rounded_rectangle([PAD, y, PAD + AW, y + h], radius=16, outline=(255, 255, 255, 30), width=1)
        if label:
            d.text((PAD + 15, y + 13), label, fill=MUTED, font=font(10, True))
        y += h + 14
        return y - h - 14

    def rrect(box, fill, outline, radius=13, width=1):
        d.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)

    # ---------------- header ----------------
    rrect([PAD, y, PAD + 40, y + 40], CYAN, None, radius=12)
    d.text((PAD + 20, y + 20), "🚗", font=F_BIG, fill=(255, 255, 255), anchor="mm")
    d.text((PAD + 54, y + 10), "4WD AUTONOMOUS CAR", fill=TEXT, font=F_TITLE)
    d.text((PAD + 54, y + 28), "ESP32-CAM · OV2640 · Arduino Uno", fill=MUTED, font=F_XS)
    d.ellipse([PAD + AW - 16, y + 14, PAD + AW - 6, y + 24], fill=GREEN)
    d.text((PAD + AW - 24, y + 17), "online", fill=MUTED, font=F_XS, anchor="ra")
    y += 62

    # ---------------- video ----------------
    cy = card(300)
    fw_x, fw_y, fw_w = PAD + 13, cy + 34, AW - 26
    fw_h = int(fw_w * 3 / 4)
    rrect([fw_x, fw_y, fw_x + fw_w, fw_y + fw_h], (5, 7, 15), (255, 255, 255, 30), radius=12)
    d.text((fw_x + fw_w // 2, fw_y + fw_h // 2), "LIVE CAMERA FEED\n320 × 240 MJPEG",
           fill=(71, 85, 105), font=F_SM, anchor="mm")
    # badges
    rrect([fw_x + 8, fw_y + 8, fw_x + 62, fw_y + 25], (0, 0, 0), (255, 255, 255, 40), radius=20)
    d.ellipse([fw_x + 15, fw_y + 14, fw_x + 21, fw_y + 20], fill=RED)
    d.text((fw_x + 27, fw_y + 16), "LIVE", fill=(254, 205, 211), font=F_XS)
    rrect([fw_x + fw_w - 66, fw_y + 8, fw_x + fw_w - 8, fw_y + 25], (0, 0, 0),
          (255, 255, 255, 40), radius=20)
    d.text((fw_x + fw_w - 37, fw_y + 16), "320×240", fill=(165, 180, 252), font=F_XS, anchor="mm")
    y = cy + 34 + fw_h + 14

    # ---------------- telemetry ----------------
    cy = card(126, "TELEMETRY")
    gx, gy, gw, gh = PAD + 13, cy + 30, (AW - 26 - 9) // 2, 58
    rrect([gx, gy, gx + gw, gy + gh], PANEL, (255, 255, 255, 22), radius=12)
    d.text((gx + 10, gy + 9), "DISTANCE", fill=MUTED, font=font(9, True))
    d.text((gx + 10, gy + 22), "37", fill=TEXT, font=F_BIG)
    d.text((gx + 40, gy + 34), "cm", fill=MUTED, font=F_XS)
    d.rounded_rectangle([gx + 10, gy + 48, gx + gw - 10, gy + 52], radius=3, fill=(255, 255, 255, 20))
    d.rounded_rectangle([gx + 10, gy + 48, gx + 10 + int((gw - 20) * 0.185), gy + 52],
                        radius=3, fill=AMBER)
    gx2 = gx + gw + 9
    rrect([gx2, gy, gx2 + gw, gy + gh], PANEL, (255, 255, 255, 22), radius=12)
    d.text((gx2 + 10, gy + 9), "SPEED / MODE", fill=MUTED, font=font(9, True))
    d.text((gx2 + 10, gy + 22), "165", fill=TEXT, font=F_BIG)
    d.rounded_rectangle([gx2 + 10, gy + 48, gx2 + gw - 10, gy + 52], radius=3, fill=(255, 255, 255, 20))
    d.rounded_rectangle([gx2 + 10, gy + 48, gx2 + 10 + int((gw - 20) * 0.647), gy + 52],
                        radius=3, fill=CYAN)
    rrect([gx, gy + gh + 9, gx + AW - 26, gy + gh + 9 + 62], PANEL, (255, 255, 255, 22), radius=12)
    d.text((gx + 10, gy + gh + 20), "IR OBSTACLE SENSORS", fill=MUTED, font=font(9, True))
    rrect([gx + 10, gy + gh + 34, gx + 92, gy + gh + 54], (34, 197, 94, 31),
          (34, 197, 94, 68), radius=20)
    d.text((gx + 51, gy + gh + 44), "LEFT clear", fill=GREEN_L, font=font(9, True), anchor="mm")
    rrect([gx + 100, gy + gh + 34, gx + 194, gy + gh + 54], (239, 68, 68, 31),
          (239, 68, 68, 85), radius=20)
    d.text((gx + 147, gy + gh + 44), "RIGHT BLOCKED", fill=(252, 165, 165), font=font(9, True), anchor="mm")

    # ---------------- driving mode ----------------
    cy = card(126, "DRIVING MODE")
    mw, mh = (AW - 26 - 9) // 2, 54
    for i, (mid, ic, nm, ds) in enumerate(modes):
        mx = PAD + 13 + (i % 2) * (mw + 9)
        my = cy + 32 + (i // 2) * (mh + 9)
        act = mid == "m-m"
        fill = (34, 197, 94, 38) if act else PANEL
        edge = GREEN if act else (255, 255, 255, 30)
        rrect([mx, my, mx + mw, my + mh], fill, edge, radius=13, width=2 if act else 1)
        d.text((mx + 10, my + 8), ic, font=F_SM)
        d.text((mx + 30, my + 9), nm, fill=GREEN_L if act else TEXT, font=F_NAME)
        d.text((mx + 30, my + 25), ds, fill=MUTED, font=F_XS)
        d.text((mx + mw - 10, my + 8), nm[0], fill=GREEN_L if act else DIM, font=font(9, True), anchor="ra")

    # ---------------- machine learning ----------------
    cy = card(104, "MACHINE LEARNING")
    for gg in range(52):
        k = gg / 51
        d.line([(PAD + 13 + gg, cy + 44), (PAD + 13 + gg, cy + 96)],
               fill=(int(139 + (34 - 139) * k), int(92 + (211 - 92) * k), int(246 + (238 - 246) * k)))
    d.rounded_rectangle([PAD + 13, cy + 32, PAD + 65, cy + 84], radius=14,
                        fill=None, outline=None)
    d.text((PAD + 39, cy + 60), "F", fill=(255, 255, 255), font=font(23, True), anchor="mm")
    d.text((PAD + 78, cy + 34), "FORWARD", fill=TEXT, font=F_NAME)
    d.text((PAD + 78, cy + 50), "confidence 94%", fill=MUTED, font=F_XS)
    d.rounded_rectangle([PAD + 78, cy + 66, PAD + AW - 13, cy + 73], radius=4, fill=(255, 255, 255, 20))
    d.rounded_rectangle([PAD + 78, cy + 66, PAD + 78 + int((AW - 91) * 0.94), cy + 73],
                        radius=4, fill=CYAN)
    d.text((PAD + AW - 15, cy + 34), "200 ms ago", fill=CYAN, font=F_XS, anchor="ra")

    # ---------------- movement ----------------
    cy = card(196, "MOVEMENT")
    pw, ph = (AW - 26 - 18) // 3, 56
    for i, (cmd, ar, lb) in enumerate(pads[:6]):
        px = PAD + 13 + (i % 3) * (pw + 9)
        py = cy + 32 + (i // 3) * (ph + 9)
        stop = cmd == "S"
        rrect([px, py, px + pw, py + ph], (127, 29, 29) if stop else PANEL,
              (239, 68, 68, 100) if stop else (255, 255, 255, 30), radius=13)
        d.text((px + pw // 2, py + 20), ar, fill=TEXT, font=font(17), anchor="mm")
        d.text((px + pw // 2, py + 40), lb, fill=(252, 165, 165) if stop else MUTED,
               font=font(9, True), anchor="mm")
    if len(pads) > 6:
        cmd, ar, lb = pads[6]
        py = cy + 32 + 2 * (ph + 9)
        rrect([PAD + 13, py, PAD + 13 + AW - 26, py + 42], PANEL, (255, 255, 255, 30), radius=13)
        d.text((PAD + 13 + (AW - 26) // 2, py + 16), ar, fill=TEXT, font=font(17), anchor="mm")
        d.text((PAD + 13 + (AW - 26) // 2, py + 33), lb, fill=MUTED, font=font(9, True), anchor="mm")

    # ---------------- speed ----------------
    cy = card(84, "SPEED")
    sx = PAD + 13
    d.rounded_rectangle([sx, cy + 40, sx + AW - 70, cy + 45], radius=3, fill=(255, 255, 255, 40))
    d.rounded_rectangle([sx, cy + 40, sx + int((AW - 70) * 0.5), cy + 45], radius=3, fill=CYAN)
    d.ellipse([sx + int((AW - 70) * 0.5) - 8, cy + 34, sx + int((AW - 70) * 0.5) + 8, cy + 50],
              fill=CYAN)
    d.text((PAD + AW - 20, cy + 42), "5", fill=CYAN, font=F_NAME, anchor="rm")

    # ---------------- dev ----------------
    cy = card(58, "")
    d.text((PAD + 15, cy + 13), "▸ FOR ML DEVELOPMENT — STREAM & API", fill=MUTED,
           font=font(10, True))
    d.text((PAD + 15, cy + 32), "http://192.168.4.1:81/stream", fill=CYAN, font=F_XS)

    # ---------------- footer ----------------
    y += 6
    d.text((W // 2, y + 6), "W A S D  move   ·   Q E  curve   ·   Space  stop   ·   M A C  modes   ·   1–9  speed",
           fill=DIM, font=F_XS, anchor="mm")
    d.text((W // 2, y + 24), "Arduino Uno + L298N + ESP32-CAM · framed protocol · 9600 baud",
           fill=DIM, font=F_XS, anchor="mm")
    d.text((W // 2, y + 46),
           "PREVIEW — drawn from the firmware's real HTML, not a browser screenshot",
           fill=(71, 85, 105), font=F_XS, anchor="mm")
    y += 62

    img = img.crop((0, 0, W, y))
    img.save(OUT)
    print(f"✅ wrote {OUT}  ({img.size[0]}x{img.size[1]})")


if __name__ == "__main__":
    main()
