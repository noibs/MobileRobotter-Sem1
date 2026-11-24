# Generates docs/line_following_flowchart.svg. Run: python docs/make_flowchart.py docs/line_following_flowchart.svg
import sys

W, H = 1480, 1700
INK = "#1e293b"
LINE = "#334155"
DATA = "#2563eb"
FONT = "Arial, Helvetica, sans-serif"
MONO = "Consolas, 'Courier New', monospace"

out = []


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def text_lines(cx, cy, lines, size=14, color=INK, weight="normal", family=FONT, lh=17):
    n = len(lines)
    y0 = cy - (n - 1) * lh / 2
    for i, line in enumerate(lines):
        out.append(
            f'<text x="{cx}" y="{y0 + i * lh:.1f}" font-family="{family}" font-size="{size}" '
            f'font-weight="{weight}" fill="{color}" text-anchor="middle" dominant-baseline="central">{esc(line)}</text>'
        )


def process(cx, y, w, h, lines, fill="#ffffff"):
    out.append(f'<rect x="{cx - w / 2}" y="{y}" width="{w}" height="{h}" rx="5" fill="{fill}" stroke="{LINE}" stroke-width="1.5"/>')
    text_lines(cx, y + h / 2, lines)


def terminator(cx, y, w, h, lines):
    out.append(f'<rect x="{cx - w / 2}" y="{y}" width="{w}" height="{h}" rx="{h / 2}" fill="#e2e8f0" stroke="{LINE}" stroke-width="1.5"/>')
    text_lines(cx, y + h / 2, lines, weight="bold")


def io(cx, y, w, h, lines):
    s = 14
    x0, x1 = cx - w / 2, cx + w / 2
    pts = f"{x0 + s},{y} {x1 + s},{y} {x1 - s},{y + h} {x0 - s},{y + h}"
    out.append(f'<polygon points="{pts}" fill="#ffffff" stroke="{LINE}" stroke-width="1.5"/>')
    text_lines(cx, y + h / 2, lines)


def decision(cx, cy, w, h, lines):
    pts = f"{cx},{cy - h / 2} {cx + w / 2},{cy} {cx},{cy + h / 2} {cx - w / 2},{cy}"
    out.append(f'<polygon points="{pts}" fill="#f1f5f9" stroke="{LINE}" stroke-width="1.5"/>')
    text_lines(cx, cy, lines, size=13, lh=16)


def path(points, arrow=True, color=LINE, dash=None, width=1.5):
    d = "M " + " L ".join(f"{x},{y}" for x, y in points)
    marker = ""
    if arrow:
        marker = ' marker-end="url(#arrow-data)"' if color == DATA else ' marker-end="url(#arrow)"'
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
    out.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}"{dash_attr}{marker}/>')


def dot(x, y):
    out.append(f'<circle cx="{x}" cy="{y}" r="3.5" fill="{LINE}"/>')


def label(x, y, s, anchor="start", color=INK, size=12, style="normal", weight="bold"):
    out.append(
        f'<text x="{x}" y="{y}" font-family="{FONT}" font-size="{size}" font-style="{style}" font-weight="{weight}" '
        f'fill="{color}" text-anchor="{anchor}" dominant-baseline="central">{esc(s)}</text>'
    )


def data_arrow(x_from, x_to, y, text):
    path([(x_from, y), (x_to, y)], color=DATA, dash="6 4")
    label((x_from + x_to) / 2, y - 10, text, anchor="middle", color=DATA, size=11.5, weight="normal")


# --- Frame -------------------------------------------------------------------
out.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
out.append(
    "<defs>"
    f'<marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="8" markerHeight="8" orient="auto-start-reverse">'
    f'<path d="M 0 0 L 10 5 L 0 10 z" fill="{LINE}"/></marker>'
    f'<marker id="arrow-data" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="8" markerHeight="8" orient="auto-start-reverse">'
    f'<path d="M 0 0 L 10 5 L 0 10 z" fill="{DATA}"/></marker>'
    "</defs>"
)
out.append(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')

lanes = [
    (30, 590, "CORE 0  ·  main program and stepping"),
    (610, 870, "SHARED MEMORY"),
    (890, 1450, "CORE 1  ·  control, about every 1 ms"),
]
for x0, x1, title in lanes:
    out.append(f'<rect x="{x0}" y="30" width="{x1 - x0}" height="1560" fill="#f8fafc" stroke="#cbd5e1" stroke-width="1"/>')
    out.append(f'<rect x="{x0}" y="30" width="{x1 - x0}" height="44" fill="{INK}"/>')
    label((x0 + x1) / 2, 52, title, anchor="middle", color="#ffffff", size=15)

# --- Core 0 ------------------------------------------------------------------
C0 = 310
terminator(C0, 100, 340, 44, ["robot.follow_line(speed, distance)"])
path([(C0, 144), (C0, 170)])
process(C0, 170, 340, 44, ["Check calibration, reset PD memory"])
path([(C0, 214), (C0, 240)])
process(C0, 240, 340, 56, ["move(): validate input,", "set start speed"])
path([(C0, 296), (C0, 322)])
process(C0, 322, 340, 56, ["First steer(), write start delays and PWM,", "RUNNING = 1"])
path([(C0, 378), (C0, 404)])
process(C0, 404, 340, 44, ["Start core 1 thread"])

# Thread start
path([(480, 426), (1000, 426)], dash="8 5")
label(740, 414, "_thread.start_new_thread()", anchor="middle", size=11.5, weight="normal")

# Stepping loop group
out.append('<rect x="50" y="466" width="510" height="574" rx="10" fill="none" stroke="#94a3b8" stroke-width="1.2" stroke-dasharray="5 4"/>')
label(552, 480, "Stepping loop", anchor="end", size=11.5, style="italic", weight="normal", color="#475569")

path([(C0, 448), (C0, 500)])
dot(C0, 485)
process(C0, 500, 340, 44, ["Read wheel delay (left, then right)"])
path([(C0, 544), (C0, 555)])
decision(C0, 600, 230, 90, ["Delay passed since", "last step?"])
label(C0 + 8, 658, "Yes")
path([(C0, 645), (C0, 680)])
process(C0, 680, 340, 56, ["Step motor, count step,", "schedule next step"])
path([(C0, 736), (C0, 790)])
# No: skip the step
path([(195, 600), (90, 600), (90, 765), (C0, 765)], arrow=False)
dot(C0, 765)
label(150, 588, "No", anchor="middle")
decision(C0, 835, 230, 90, ["Core 1 heartbeat", "within 100 ms?"])
label(C0 + 8, 893, "Yes")
path([(C0, 880), (C0, 930)])
decision(C0, 975, 230, 90, ["RUNNING = 1?"])
# Yes: loop back
path([(195, 975), (70, 975), (70, 485), (C0, 485)], arrow=False)
label(140, 963, "Yes", anchor="middle")
# Watchdog failed
path([(425, 835), (530, 835), (530, 1225), (C0, 1225)], arrow=False)
label(478, 823, "No: error", anchor="middle")
# Loop ends
label(C0 + 8, 1035, "No")
path([(C0, 1020), (C0, 1240)])
dot(C0, 1225)
process(C0, 1240, 340, 56, ["RUNNING = 0, wait until", "core 1 sets CORE1_DONE"])
path([(C0, 1296), (C0, 1315)])
decision(C0, 1360, 230, 90, ["End speed = 0", "or error?"])
label(C0 + 8, 1418, "Yes")
path([(C0, 1405), (C0, 1440)])
process(C0, 1440, 340, 44, ["Stop motors, speed = 0"])
path([(C0, 1484), (C0, 1540)])
path([(425, 1360), (520, 1360), (520, 1520), (C0, 1520)], arrow=False)
dot(C0, 1520)
label(470, 1348, "No", anchor="middle")
label(527, 1440, "keep speed for", anchor="start", size=11, weight="normal", style="italic", color="#475569")
label(527, 1455, "the next move", anchor="start", size=11, weight="normal", style="italic", color="#475569")
terminator(C0, 1540, 340, 44, ["Return to the script"])

# --- Core 1 ------------------------------------------------------------------
C1 = 1170
terminator(C1, 404, 340, 44, ["Core 1 thread starts"])
path([(C1, 448), (C1, 500)])
dot(C1, 485)
io(C1, 500, 330, 56, ["steer(): read 8 line sensors,", "position e from −1 to 1"])
path([(C1, 556), (C1, 565)])
decision(C1, 610, 170, 90, ["Line seen?"])
label(C1 + 8, 668, "Yes")
path([(C1, 655), (C1, 690)])
process(C1, 690, 300, 56, ["Every 1 cm driven:", "D = Δe / Δs"])
path([(C1, 746), (C1, 790)])
# No: line lost
path([(1085, 610), (1047, 610)])
label(1066, 596, "No", anchor="middle")
process(972, 582, 150, 56, ["e = last side (±1),", "D = 0"])
path([(972, 638), (972, 765), (C1, 765)], arrow=False)
dot(C1, 765)
process(C1, 790, 340, 56, ["correction = Kp·e + Kd·D", "factors = 1 ± correction"])
path([(C1, 846), (C1, 872)])
process(C1, 872, 340, 44, ["Measure dt, heartbeat + 1"])
path([(C1, 916), (C1, 946)])
process(C1, 946, 340, 56, ["Base speed target:", "min(v_cruise, √(v_end² + 2·a·s_left))"])
path([(C1, 1002), (C1, 1026)])
process(C1, 1026, 340, 44, ["Ramp base speed by acceleration · dt"])
path([(C1, 1070), (C1, 1094)])
process(C1, 1094, 340, 56, ["Wheel speed = factor · base speed,", "ramped by wheel acceleration · dt"])
path([(C1, 1150), (C1, 1174)])
process(C1, 1174, 340, 56, ["Convert to step delay and PWM,", "write to shared memory"])
path([(C1, 1230), (C1, 1255)])
decision(C1, 1300, 230, 90, ["Distance driven", "or RUNNING = 0?"])
# No: loop back
path([(1285, 1300), (1420, 1300), (1420, 485), (C1, 485)], arrow=False)
label(1350, 1288, "No", anchor="middle")
label(C1 + 8, 1358, "Yes")
path([(C1, 1345), (C1, 1400)])
terminator(C1, 1400, 340, 56, ["Core 1 done:", "RUNNING = 0, CORE1_DONE = 1"])

# --- Shared memory -----------------------------------------------------------
SX0, SX1 = 625, 855
SC = (SX0 + SX1) / 2
out.append(
    f'<path d="M {SX0},500 L {SX0},1470 A {(SX1 - SX0) / 2},16 0 0 0 {SX1},1470 L {SX1},500" '
    f'fill="#eff6ff" stroke="{DATA}" stroke-width="1.5"/>'
)
out.append(f'<ellipse cx="{SC}" cy="500" rx="{(SX1 - SX0) / 2}" ry="16" fill="#dbeafe" stroke="{DATA}" stroke-width="1.5"/>')
text_lines(SC, 548, ["Shared array", "11 × int32"], size=14, weight="bold", color=INK)
fields = [
    ("RUNNING", "both cores"),
    ("LEFT / RIGHT_DELAY", "core 1 → core 0"),
    ("LEFT / RIGHT_PWM", "core 1 → core 0"),
    ("LEFT / RIGHT_STEPS", "core 0 → core 1"),
    ("HEARTBEAT", "core 1 → core 0"),
    ("CORE1_DONE", "core 1 → core 0"),
    ("MAX_LATE_US", "core 0 (jitter)"),
    ("SLOWEST_LOOP_US", "core 1 (loop time)"),
]
y = 610
for name, who in fields:
    label(SC, y, name, anchor="middle", size=12.5, weight="bold")
    label(SC, y + 15, who, anchor="middle", size=11, weight="normal", color="#475569")
    y += 40
text_lines(SC, 960, ["One writer per value.", "32-bit writes are atomic", "on the RP2040, so no", "locks are needed."],
           size=11.5, color="#475569", lh=15)

# Data flows (dashed blue)
data_arrow(SX0, 480, 522, "delays, PWM")
data_arrow(480, SX0, 708, "step counts")
data_arrow(SX0, 361, 810, "heartbeat")
data_arrow(SX0, 480, 1268, "CORE1_DONE")
data_arrow(1000, SX1, 894, "heartbeat")
data_arrow(1000, SX1, 1202, "delays, PWM")
data_arrow(SX1, 1119, 1275, "step counts")
data_arrow(1000, SX1, 1428, "done flags")

# --- Legend ------------------------------------------------------------------
LY = 1622
label(40, LY, "Legend", size=13)
out.append(f'<rect x="120" y="{LY - 13}" width="46" height="26" rx="13" fill="#e2e8f0" stroke="{LINE}" stroke-width="1.2"/>')
label(174, LY, "start / end", weight="normal")
out.append(f'<rect x="270" y="{LY - 13}" width="46" height="26" rx="4" fill="#ffffff" stroke="{LINE}" stroke-width="1.2"/>')
label(324, LY, "process", weight="normal")
out.append(f'<polygon points="420,{LY - 15} 446,{LY} 420,{LY + 15} 394,{LY}" fill="#f1f5f9" stroke="{LINE}" stroke-width="1.2"/>')
label(454, LY, "decision", weight="normal")
out.append(f'<polygon points="540,{LY - 13} 590,{LY - 13} 580,{LY + 13} 530,{LY + 13}" fill="#ffffff" stroke="{LINE}" stroke-width="1.2"/>')
label(598, LY, "sensor input", weight="normal")
path([(700, LY), (750, LY)])
label(758, LY, "program flow", weight="normal")
path([(870, LY), (920, LY)], color=DATA, dash="6 4")
label(928, LY, "data through shared memory", weight="normal")
path([(1120, LY), (1170, LY)], dash="8 5")
label(1178, LY, "starts a thread", weight="normal")
dot(1300, LY)
label(1312, LY, "paths join", weight="normal")

out.append("</svg>")
open(sys.argv[1], "w", encoding="utf-8").write("\n".join(out))
print("written", sys.argv[1])
