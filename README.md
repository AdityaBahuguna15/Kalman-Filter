# Scalable Linear Kalman Filter (1D / 2D / 3D)
A flexible, modular Python implementation of a Linear Kalman Filter. This project is designed to automatically scale its matrix operations to handle 1D, 2D, or 3D tracking scenarios without changing the core logic.

It includes a Synthetic Data Generator to stress-test the filter against various non-linear paths (Circles, Spirals, Figure-8s) and allows toggling between Active Control (Robot mode) and Passive Tracking (Radar mode).

## Features

- Automatic matrix scaling for 1D/2D/3D tracking
- Dual modes: Active control (robot) and passive tracking (radar)
- Synthetic trajectory generation (linear, circle, sine, figure-8, spiral)
- Automatic visualization for all dimensions

## Project Structure

```
├── Filter.py          # LinearKalmanFilter class
├── datagen.py         # Trajectory generation, execution, visualization
```

## Quick Start

Configure in `datagen.py`:
```python
SELECTED_SHAPE = 'spiral'    # 'linear', 'circle', 'sine', 'figure8', 'spiral'
SELECTED_DIM = 2             # 1, 2, or 3
TIMESTEP = 0.1               # Time between measurements
CONTROL = False              # True: Active, False: Passive
```

## Usage in Your Code

```python
import numpy as np
from Filter import LinearKalmanFilter

kf = LinearKalmanFilter()
measurements = np.load("sensor_data.npy")  # Shape: (timesteps, dim)

estimates = kf.filter(
    measurements=measurements,
    dim=2,
    dt=0.05,
    ActiveControl=False
)
```

## Theory

**State Vector**: Tracks position and velocity per dimension
- 1D: `[p, v]ᵀ` (size 2)
- 2D: `[px, py, vx, vy]ᵀ` (size 4)
- 3D: `[px, py, pz, vx, vy, vz]ᵀ` (size 6)

**Key Matrices** (auto-scaled):
- `A`: State transition (constant velocity model)
- `H`: Measurement (extracts position)
- `Q`: Process noise (model trust)
- `R`: Measurement noise (sensor trust)

## Tuning Parameters (in Filter.py)

**Process Noise (Q)**: Default = 0.1
- Lower (0.01): Trust model → smoother, more lag
- Higher (1.0): Distrust model → responsive, noisier

**Measurement Noise (R)**: Default = 10.0
- Lower (1.0): Trust sensor → track measurements closely
- Higher (100.0): Distrust sensor → heavy smoothing

**Rule of Thumb**: `Q/R` ratio determines behavior
- `Q/R << 1`: Smooth output, slow response
- `Q/R >> 1`: Fast response, noisy output

## Modes

**Passive (CONTROL=False)**:
- No control input (`u=0`)
- Relies on measurements only
- Exhibits lag during acceleration

**Active (CONTROL=True)**:
- Uses control input for prediction
- Reduces lag with known accelerations
- Requires control signal `u`

** Key Note **:
Using Active control will actually increase RSME due to the physics model being used. WHY? For every loop, the filter predicts the object has moved further ahead due to an acceleration of 1m/s^2 (u = 1). However Measurement (which hasnt accelerated) says the object is behind. This constant tug-pull between false prediction and true measurement causes lag, increasing RSME. FIX: Will need to modify the data generator to use a model using the equation: $p_{new} = p + v \cdot dt + \frac{1}{2} a \cdot dt^2$, simulating acceleration.
```python
# An example of data with REAL control input
pos = np.zeros(dim)
vel = np.zeros(dim)
u_actual = np.ones(dim) * 1.0  # The real acceleration

for k in range(steps):
    pos = pos + vel * dt + 0.5 * u_actual * (dt**2)
    vel = vel + u_actual * dt
    TruePath[k] = pos
```
## Troubleshooting

| Issue | Solution |
|-------|----------|
| Filter lags behind | Increase Q or decrease R |
| Output is noisy | Decrease Q or increase R |
| Filter diverges | Check Q, R are positive definite |
| Poor on turns | Increase Q or enable active control |


**Last Updated**: February 2026  
