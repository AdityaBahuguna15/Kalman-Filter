import numpy as np
import matplotlib.pyplot as plt

def compare_filters():
    # SETUP DATA
    dt = 0.1
    steps = 100
    TruePath = np.zeros(steps)
    measurements = np.zeros(steps)
    
    # Create a "Sudden Turn" scenario
    true_pos = 0
    velocity = 2
    
    for k in range(steps):
        if k == 50: velocity = -2 # SUDDEN DIRECTION CHANGE
        true_pos += velocity * dt
        TruePath[k] = true_pos
        measurements[k] = true_pos + np.random.randn() * 2.0 # Noisy sensor

    # FILTER FUNCTION
    def run_filter(r_val, q_val):
        x = np.array([0., 0.]) # pos, vel
        P = np.eye(2) * 10
        F = np.array([[1, dt], [0, 1]])
        H = np.array([[1, 0]])
        R = np.array([[r_val**2]]) 
        Q = np.array([[0, 0], [0, q_val**2]]) # Simplified Q
        
        estimates = []
        for z in measurements:
            # Predict
            x = F @ x
            P = F @ P @ F.T + Q
            # Update
            y = z - H @ x
            S = H @ P @ H.T + R
            K = P @ H.T @ np.linalg.inv(S)
            x = x + K @ y
            P = (np.eye(2) - K @ H) @ P
            estimates.append(x[0])
        return estimates

    # TWO COMPETITORS
    # Filter A: "The Smooth Operator" (High R, Low Q)
    est_smooth = run_filter(r_val=5.0, q_val=0.01)

    # Filter B: "The Nervous React" (Low R, High Q)
    est_fast = run_filter(r_val=0.5, q_val=2.0)

    # 4. PLOT
    plt.figure(figsize=(10, 6))
    plt.plot(TruePath, 'k-', linewidth=3, alpha=0.3, label='Truth (Sudden Turn)')
    plt.scatter(range(steps), measurements, c='r', s=10, alpha=0.3, label='Measurements')
    
    plt.plot(est_smooth, 'b-', linewidth=2, label='Strong Model (Smooth but Laggy)')
    plt.plot(est_fast, 'orange', linewidth=2, label='Strong Sensor (Fast but Jittery)')
    
    plt.legend()
    plt.title("Tuning the 'Strength': Smoothness vs. Responsiveness")
    plt.grid()
    plt.show()

compare_filters()