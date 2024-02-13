import matplotlib.pyplot as plt
import numpy as np

# set three different t_span values
t_span_values = [30, 60, 90]
freq = 100   # hz

# iterate each t_span
for i, t_span in enumerate(t_span_values):
    t = np.linspace(0, t_span, t_span * freq)  # time array

    d_x = np.cos(0.1 * t)
    d_y = np.sin(0.12 * t)
    d_z = np.sin(0.08 * t)

    Ap_x = 0.25 * np.cos(t) + np.cos(0.1 * t)
    Ap_y = 0.25 * np.sin(t) + np.sin(0.12 * t)
    Ap_z = np.sin(0.08 * t)

    # create subplot
    plt.subplot(1, len(t_span_values), i + 1, projection='3d')

    # Robot position
    plt.plot(d_x, d_y, d_z, label='Robot')

    # A_p(t) position
    plt.plot(Ap_x, Ap_y, Ap_z, label='p(t)')

    # Labels and title
    plt.xlabel('X axis')
    plt.ylabel('Y axis')
    plt.clabel('Z axis')
    plt.title(f't_span = {t_span}s')
    plt.legend()

plt.tight_layout()
plt.show()