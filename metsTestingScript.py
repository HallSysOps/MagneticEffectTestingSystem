import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# === CONFIGURATION ===
port = 'COM7'        # <-- Change if needed
baud = 9600
PLOT_WINDOW = 200    # Number of points shown in live plot

# === SETUP SERIAL ===
ser = serial.Serial(port, baud, timeout=1)
print(f"Connected to {port}")

# === DATA BUFFERS ===
r_vals = []
r_avg_vals = []
theta_vals = []
theta_avg_vals = []
phi_vals = []
phi_avg_vals = []

# === SETUP PLOTS ===
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 6))
fig.suptitle('Real-Time Magnetometer Readings')

def init_axes():
    for ax, label in zip((ax1, ax2, ax3), ('r (magnitude)', 'theta (deg)', 'phi (deg)')):
        ax.set_ylim(-180, 180)
        ax.set_ylabel(label)
        ax.grid(True)
    ax3.set_xlabel('Sample Index')

# === PLOT LINES ===
# Blue: live data, Red: average, Black: ±1 deflection
lines = [
    ax1.plot([], [], 'b-', label='r', antialiased=False)[0],
    ax1.plot([], [], 'r--', label='rAvg', antialiased=False)[0],
    ax2.plot([], [], 'b-', label='θ', antialiased=False)[0],
    ax2.plot([], [], 'r--', label='θAvg', antialiased=False)[0],
    ax3.plot([], [], 'b-', label='φ', antialiased=False)[0],
    ax3.plot([], [], 'r--', label='φAvg', antialiased=False)[0],
]

# ±1 deflection threshold lines
deflection_lines = [
    ax1.plot([], [], 'k:', label='±1')[0], ax1.plot([], [], 'k:')[0],
    ax2.plot([], [], 'k:', label='±1')[0], ax2.plot([], [], 'k:')[0],
    ax3.plot([], [], 'k:', label='±1')[0], ax3.plot([], [], 'k:')[0],
]

# === ANIMATION UPDATE ===
def update_plot(frame):
    if ser.in_waiting == 0:
        return lines + deflection_lines  # No data, skip update safely

    line = ser.readline().decode('utf-8').strip()
    if not line:
        return lines + deflection_lines  # Skip empty lines

    try:
        parts = [float(p.strip()) for p in line.split(',')]
        if len(parts) == 9:
            r, rAvg, _, theta, thetaAvg, _, phi, phiAvg, _ = parts

            r_vals.append(r)
            r_avg_vals.append(rAvg)
            theta_vals.append(theta)
            theta_avg_vals.append(thetaAvg)
            phi_vals.append(phi)
            phi_avg_vals.append(phiAvg)

            # Determine range of data to show
            def last_n(data): return data[-PLOT_WINDOW:]
            def idx_range(data): return range(max(0, len(data) - PLOT_WINDOW), len(data))

            # Plot r
            x = idx_range(r_vals)
            lines[0].set_data(x, last_n(r_vals))
            lines[1].set_data(x, last_n(r_avg_vals))
            if r_avg_vals:
                r_avg = r_avg_vals[-1]
                deflection_lines[0].set_data(x, [r_avg + 1] * len(x))
                deflection_lines[1].set_data(x, [r_avg - 1] * len(x))
                ax1.set_ylim(r_avg - 2, r_avg + 2)
                ax1.set_xlim(x.start, x.stop)

            # Plot theta
            x = idx_range(theta_vals)
            lines[2].set_data(x, last_n(theta_vals))
            lines[3].set_data(x, last_n(theta_avg_vals))
            if theta_avg_vals:
                theta_avg = theta_avg_vals[-1]
                deflection_lines[2].set_data(x, [theta_avg + 1] * len(x))
                deflection_lines[3].set_data(x, [theta_avg - 1] * len(x))
                ax2.set_ylim(theta_avg - 2, theta_avg + 2)
                ax2.set_xlim(x.start, x.stop)

            # Plot phi
            x = idx_range(phi_vals)
            lines[4].set_data(x, last_n(phi_vals))
            lines[5].set_data(x, last_n(phi_avg_vals))
            if phi_avg_vals:
                phi_avg = phi_avg_vals[-1]
                deflection_lines[4].set_data(x, [phi_avg + 1] * len(x))
                deflection_lines[5].set_data(x, [phi_avg - 1] * len(x))
                ax3.set_ylim(phi_avg - 2, phi_avg + 2)
                ax3.set_xlim(x.start, x.stop)

    except ValueError:
        pass  # Ignore malformed lines

    return lines + deflection_lines

# === INIT + RUN ===
init_axes()
for ax in (ax1, ax2, ax3):
    ax.legend(loc='upper right')

ani = animation.FuncAnimation(fig, update_plot, interval=100)
plt.tight_layout()
plt.show()
