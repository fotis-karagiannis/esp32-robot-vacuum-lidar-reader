import serial
import math
import re
import collections
import matplotlib.pyplot as plt
import numpy as np

# Config
SERIAL_PORT = "COM4"      # Change this to your ESP32 port
BAUD_RATE = 115200
MAX_POINTS = 2000         # More points = smoother map
SMOOTH_WINDOW = 5         # Moving average window size

# Regex to parse ESP32 output
pattern = re.compile(
    r"Angle=([\d\.]+) deg, Distance=(\d+) mm, Strength=(\d+)"
)

# Serial
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

# Buffers
angles_deg = collections.deque(maxlen=MAX_POINTS)
distances_raw = collections.deque(maxlen=MAX_POINTS)
strengths = collections.deque(maxlen=MAX_POINTS)

# Smoothing
def moving_average(data, window):
    if len(data) < window:
        return list(data)
    cumsum = np.cumsum(np.insert(data, 0, 0))
    smoothed = (cumsum[window:] - cumsum[:-window]) / float(window)
    pad = [smoothed[0]] * (len(data) - len(smoothed))
    return pad + list(smoothed)

# Matplotlib setup
plt.ion()
fig = plt.figure(figsize=(12, 6))

# Subplot 1: Polar (radar)
ax_polar = fig.add_subplot(1, 2, 1, polar=True)
ax_polar.set_title("Polar (Radar)")
ax_polar.set_theta_zero_location("N")
ax_polar.set_theta_direction(-1)

# Subplot 2: Cartesian X/Y
ax_cart = fig.add_subplot(1, 2, 2)
ax_cart.set_title("Cartesian Map (X/Y)")
ax_cart.set_xlabel("X (mm)")
ax_cart.set_ylabel("Y (mm)")
ax_cart.set_aspect("equal", "box")

plt.tight_layout()

# Main loop
while True:
    line = ser.readline().decode(errors="ignore").strip()

    if not line:
        continue

    match = pattern.search(line)
    if not match:
        continue

    angle_deg = float(match.group(1))
    distance = int(match.group(2))
    strength = int(match.group(3))

    # Store raw values
    angles_deg.append(angle_deg)
    distances_raw.append(distance)
    strengths.append(strength)

    # Sort by angle
    sorted_data = sorted(
        zip(angles_deg, distances_raw, strengths),
        key=lambda x: x[0]
    )

    angles_sorted, distances_sorted, strengths_sorted = zip(*sorted_data)

    # Convert to radians
    angles_rad_sorted = [math.radians(a) for a in angles_sorted]

    # Smooth distances
    distances_smooth = moving_average(list(distances_sorted), SMOOTH_WINDOW)

    # Cartesian conversion
    xs = [d * math.cos(math.radians(a)) for a, d in zip(angles_sorted, distances_smooth)]
    ys = [d * math.sin(math.radians(a)) for a, d in zip(angles_sorted, distances_smooth)]

    # ---------- UPDATE POLAR ----------
    ax_polar.clear()
    ax_polar.set_title("Polar (Radar)")
    ax_polar.set_theta_zero_location("N")
    ax_polar.set_theta_direction(-1)

    if distances_sorted:
        rmax = max(distances_sorted) + 200
        ax_polar.set_rmax(rmax)

    ax_polar.scatter(
        angles_rad_sorted,
        distances_sorted,
        c=strengths_sorted,
        cmap="viridis",
        s=10
    )

    # Update Cartesian
    ax_cart.clear()
    ax_cart.set_title("Cartesian Map (X/Y)")
    ax_cart.set_xlabel("X (mm)")
    ax_cart.set_ylabel("Y (mm)")
    ax_cart.set_aspect("equal", "box")

    if xs and ys:
        ax_cart.scatter(xs, ys, c=strengths_sorted, cmap="viridis", s=10)
        margin = 200
        ax_cart.set_xlim(min(xs) - margin, max(xs) + margin)
        ax_cart.set_ylim(min(ys) - margin, max(ys) + margin)

    plt.pause(0.001)
