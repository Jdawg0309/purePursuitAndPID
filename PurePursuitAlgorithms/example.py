import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def simulate_pure_pursuit(path, Ld, L, v, dt, num_steps):
    x, y, theta = path[0,0], path[0,1], 0.0
    robot_states = []
    current_index = 0
    for _ in range(num_steps):
        # 1. Find lookahead target
        target = path[-1]
        for i in range(current_index, len(path)):
            if np.hypot(path[i,0] - x, path[i,1] - y) >= Ld:
                target = path[i]
                current_index = i
                break

        # 2. Transform target to robot local frame
        dx = target[0] - x
        dy = target[1] - y
        local_x = dx * np.cos(theta) + dy * np.sin(theta)
        local_y = -dx * np.sin(theta) + dy * np.cos(theta)

        # 3. Calculate steering angle δ
        alpha = np.arctan2(local_y, local_x)
        delta = np.arctan((2 * L * np.sin(alpha)) / Ld)
        
        # 4. Move robot
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        theta += v / L * np.tan(delta) * dt
        
        robot_states.append((x, y))
    return np.array(robot_states)

# Parameters
Ld = 0.5        # lookahead distance
L = 0.2         # wheelbase
v = 0.5         # speed
dt = 0.1        # time step
num_steps = 400

# Example 1: Circular path
theta_vals = np.linspace(0, 2*np.pi, 200)
circle_path = np.column_stack((2.0*np.cos(theta_vals), 2.0*np.sin(theta_vals)))
circle_states = simulate_pure_pursuit(circle_path, Ld, L, v, dt, num_steps)

fig1, ax1 = plt.subplots()
ax1.plot(circle_path[:,0], circle_path[:,1])
robot_dot1, = ax1.plot([], [], marker='o')
ax1.set_title("Circular Path")

def init1():
    ax1.set_aspect('equal', 'box')
    ax1.set_xlim(-3, 3)
    ax1.set_ylim(-3, 3)
    robot_dot1.set_data([], [])
    return robot_dot1,

def update1(frame):
    robot_dot1.set_data(circle_states[frame,0], circle_states[frame,1])
    return robot_dot1,

ani1 = FuncAnimation(fig1, update1, frames=len(circle_states),
                     init_func=init1, interval=50, blit=True)


# Example 2: Sinusoidal path
t = np.linspace(0, 4*np.pi, 400)
sine_path = np.column_stack((t, np.sin(t)))
sine_states = simulate_pure_pursuit(sine_path, Ld, L, v, dt, num_steps)

fig2, ax2 = plt.subplots()
ax2.plot(sine_path[:,0], sine_path[:,1])
robot_dot2, = ax2.plot([], [], marker='o')
ax2.set_title("Sinusoidal Path")

def init2():
    ax2.set_aspect('equal', 'box')
    ax2.set_xlim(0, 4*np.pi)
    ax2.set_ylim(-2, 2)
    robot_dot2.set_data([], [])
    return robot_dot2,

def update2(frame):
    robot_dot2.set_data(sine_states[frame,0], sine_states[frame,1])
    return robot_dot2,

ani2 = FuncAnimation(fig2, update2, frames=len(sine_states),
                     init_func=init2, interval=50, blit=True)

plt.show()
