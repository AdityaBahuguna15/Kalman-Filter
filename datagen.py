import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from Filter import LinearKalmanFilter

Lkf = LinearKalmanFilter()

def generate_data(shape='linear', dim=2, steps=100, meas_noise=1.0):
    """
    Generates Ground Truth and Noisy Measurements for a specific shape and dimension.
    """
    dt = 0.1
    t = np.linspace(0, steps * dt, steps)
    
    TruePath = np.zeros((steps, dim))
    measurements = np.zeros((steps, dim))

    if shape == 'linear':
        # Constant velocity movement
        vel = np.ones(dim) * 2.0
        for k in range(steps):
            TruePath[k] = vel * (k * dt)

    elif shape == 'circle':
        # Circular motion (Standard for 2D/3D)
        radius = 20
        w = 0.5  # Angular velocity
        
        # X and Y are sinusoidal
        TruePath[:, 0] = radius * np.cos(w * t)
        if dim >= 2:
            TruePath[:, 1] = radius * np.sin(w * t)
        if dim == 3:
            TruePath[:, 2] = t * 2  # Rise in Z (Helix)

    elif shape == 'sine':
        # Sine wave trajectory
        TruePath[:, 0] = t * 5  # Linear X
        if dim >= 2:
            TruePath[:, 1] = 10 * np.sin(0.5 * t) # Oscillate Y
        if dim == 3:
            TruePath[:, 2] = t * 2 # Linear Z

    elif shape == 'figure8':
        # Lemniscate (Figure-8)
        scale = 20
        TruePath[:, 0] = scale * np.sin(t * 0.5) 
        if dim >= 2:
            TruePath[:, 1] = scale * np.sin(t * 0.5) * np.cos(t * 0.5)
        if dim == 3:
            TruePath[:, 2] = t  # Linear Z
    
    elif shape == 'spiral':
        # Archimedean Spiral: Radius grows with time
        # r = a + b * theta
        growth_rate = 0.5 
        w = 1.0 # Angular velocity (rad/s)
        
        # Calculate changing radius
        r = growth_rate * t 
        
        TruePath[:, 0] = r * np.cos(w * t) # X
        if dim >= 2:
            TruePath[:, 1] = r * np.sin(w * t) # Y
        if dim == 3:
            TruePath[:, 2] = t # Z (Conical Spiral / Tornado shape)
    
    # --- ADD NOISE (Simulate Sensor) ---
    # We add random Gaussian noise to the perfect "Ground Truth"
    noise = np.random.randn(steps, dim) * meas_noise
    measurements = TruePath + noise

    return TruePath, measurements

def plot_results(dim, shape, TruePath, measurements, estimated_path):
    rmse = np.sqrt(np.mean((TruePath - estimated_path)**2))
    print(f"Scenario: {shape.upper()} | Dimension: {dim}D | RMSE: {rmse:.2f}m")

    if dim == 1:
        # 1D: Position vs Time
        t = np.arange(len(TruePath))
        plt.figure(figsize=(10, 5))
        plt.plot(t, TruePath[:, 0], 'g-', label='True Path', linewidth=2)
        plt.scatter(t, measurements[:, 0], c='r', s=15, alpha=0.5, label='Measurements')
        plt.plot(t, estimated_path[:, 0], 'b--', label='Kalman Filter', linewidth=2)
        plt.title(f"1D Tracking: {shape} (RMSE: {rmse:.2f})")
        plt.xlabel("Time Step")
        plt.ylabel("Position")

    elif dim == 2:
        # 2D: X vs Y
        plt.figure(figsize=(8, 8))
        plt.plot(TruePath[:, 0], TruePath[:, 1], 'g-', label='True Path')
        plt.scatter(measurements[:, 0], measurements[:, 1], c='r', s=15, alpha=0.5, label='Measurements')
        plt.plot(estimated_path[:, 0], estimated_path[:, 1], 'b--', label='Kalman Filter')
        plt.title(f"2D Tracking: {shape} (RMSE: {rmse:.2f})")
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.axis('equal')

    elif dim == 3:
        # 3D: X, Y, Z
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(TruePath[:, 0], TruePath[:, 1], TruePath[:, 2], 'g-', label='True Path', linewidth=2)
        ax.scatter(measurements[:, 0], measurements[:, 1], measurements[:, 2], c='r', s=15, alpha=0.3, label='Measurements')
        ax.plot(estimated_path[:, 0], estimated_path[:, 1], estimated_path[:, 2], 'b--', label='Kalman Filter', linewidth=2)
        ax.set_title(f"3D Tracking: {shape} (RMSE: {rmse:.2f})")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    # --- CONFIGURATION ---
    # Try changing these!
    # SHAPES: 'linear', 'circle', 'sine', 'figure8'
    # DIMS: 1, 2, or 3
    
    SELECTED_SHAPE = 'spiral'
    SELECTED_DIM = 3
    TIMESTEP = 0.1
    CONTROL = False
    
    # 1. Generate Data
    gt, meas = generate_data(shape=SELECTED_SHAPE, dim=SELECTED_DIM, steps=600)
    
    # 2. Run Filter
    est = Lkf.filter(measurements=meas, dim=SELECTED_DIM, dt = TIMESTEP, ActiveControl= CONTROL)
    
    # 3. Visualize
    plot_results(SELECTED_DIM, SELECTED_SHAPE, gt, meas, est)