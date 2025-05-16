import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

def simulate_and_snapshots(path, Ld, L, v, dt, snapshots):
    x, y, theta = path[0,0], path[0,1], 0.0
    snapshot_data = []
    current_index = 0
    total_steps = max(snapshots) + 1
    
    for i in range(total_steps):
        # 1. Find lookahead target
        target = path[-1]
        for j in range(current_index, len(path)):
            if np.hypot(path[j,0] - x, path[j,1] - y) >= Ld:
                target = path[j]
                current_index = j
                break

        # 2. Transform to local frame & compute alpha
        dx, dy = target[0] - x, target[1] - y
        local_x = dx * np.cos(theta) + dy * np.sin(theta)
        local_y = -dx * np.sin(theta) + dy * np.cos(theta)
        alpha = np.arctan2(local_y, local_x)

        # 3. Compute curvature & circle center
        kappa = 2 * np.sin(alpha) / Ld
        R = abs(1.0 / kappa) if abs(kappa) > 1e-6 else 1e6
        sign = np.sign(kappa) if kappa != 0 else 1
        cx = x - sign * R * np.sin(theta)
        cy = y + sign * R * np.cos(theta)

        # Save snapshot if needed
        if i in snapshots:
            snapshot_data.append((i, x, y, target, (cx, cy), R))

        # 4. Move robot
        delta = np.arctan((2 * L * np.sin(alpha)) / Ld)
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        theta += (v / L) * np.tan(delta) * dt

    return snapshot_data

# Example path & parameters
t = np.linspace(0, 4*np.pi, 200)
path = np.column_stack((t, np.sin(t)))
Ld, L, v, dt = 0.5, 0.2, 0.5, 0.1

# Take snapshots at steps 0, 10, and 20
snapshots = simulate_and_snapshots(path, Ld, L, v, dt, [0, 10, 20])

# Plot each snapshot as a separate figure
for step, x, y, target, center, R in snapshots:
    fig, ax = plt.subplots()
    ax.plot(path[:,0], path[:,1])
    ax.plot(x, y, marker='o', label='Robot')
    ax.plot(target[0], target[1], marker='x', label='Lookahead')
    ax.plot([x, target[0]], [y, target[1]], label='Line to Target')
    
    circle = Circle(center, R, fill=False)
    ax.add_patch(circle)
    ax.set_aspect('equal', 'box')
    ax.set_title(f"Pure Pursuit at Step {step}")
    ax.legend()

plt.show()
