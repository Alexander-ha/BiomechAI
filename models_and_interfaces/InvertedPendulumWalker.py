import numpy as np
import control
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
# Simulate the system using odeint
from scipy.integrate import odeint

# Define the system parameters
m = 70  # Mass of the pendulum
M = 1  # Mass of the cart
L = 0.8  # Length to the pendulum center of mass
g = 9.81  # Acceleration due to gravity

# Linearized state-space model of the inverted pendulum system
A = np.array([[0, 1, 0, 0],
              [0, 0, -m*g/M, 0],
              [0, 0, 0, 1],
              [0, 0, g*(m+M)/(L*M), 0]])
B = np.array([[0], [1/M], [0], [-1/(L*M)]])
C = np.eye(4)
D = np.zeros((4, 1))

# Design the LQR controller
Q = np.matrix(	[
					[100,0,0,0],
					[0,100,0,0],
					[0,0,100,0],
					[0,0,0,100]
					])
R = np.array([[0.1]])
K, S, E = control.lqr(A, B, Q, R)

# Define the control input function using the LQR controller
def control_input(x):
    return -np.dot(K, x)
T = 40
# Simulate the response of the inverted pendulum system
t = np.linspace(0, T, 1000)  # Time vector
dt = T/1000
x0 = np.array([0, 0, np.pi/10, 0])  # Initial state vector


def inverted_pendulum(x, t, x0):
    x0 = x0[0]
    u = control_input(x)
    u = u[0] + m*L**2/2*np.sign(x[0]-x0)
    
    dxdt = np.dot(A, x) + np.dot(B, [u])
    return dxdt

x = odeint(inverted_pendulum, x0, t, args = (x0, ))

# Plot the results
plt.figure()
plt.plot(t, x[:, 0], label='Position')
plt.plot(t, x[:, 1], label='Velocity')
plt.plot(t, x[:, 2], label='Angle')
plt.plot(t, x[:, 3], label='Angular velocity ')
plt.xlabel('Time')
plt.ylabel('Space of states')
plt.legend()
plt.title('LQR control')
plt.show()



ths = x[:, 0]
xs = x[:, 2]


pxs = L * np.sin(ths) + xs
pys = L * np.cos(ths)

fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1.5, 1.5), ylim=(-0.5, 2))
ax.set_aspect('equal')
ax.grid()

patch = ax.add_patch(Rectangle((0, 0), 0, 0, linewidth=1, edgecolor='k', facecolor='g'))

line, = ax.plot([], [], 'o-', lw=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

cart_width = 0.2
cart_height = 0.1

def init():
    line.set_data([], [])
    time_text.set_text('')
    patch.set_xy((-cart_width/2, -cart_height/2))
    patch.set_width(cart_width)
    patch.set_height(cart_height)
    return line, time_text, patch


def animate(i):
    thisx = [xs[i], pxs[i]]
    thisy = [0, pys[i]]

    line.set_data(thisx, thisy)
    time_text.set_text(time_template % (i*dt))
    patch.set_x(xs[i] - cart_width/2)
    return line, time_text, patch

ani = animation.FuncAnimation(fig, animate, np.arange(1, len(x)),
                              interval=25, blit=True, init_func=init)



# Set up formatting for the movie files
print("Writing video...")
#Writer = animation.writers['imagemagick']
#writer = Writer(fps=25, metadata=dict(artist='Sergey Royz'), bitrate=1800)
ani.save('gait_CIP_simulation.gif')
