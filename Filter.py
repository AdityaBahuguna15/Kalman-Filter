import numpy as np

class LinearKalmanFilter:
    @staticmethod
    def filter(measurements, dim, dt=0.1, ActiveControl = False):
        # State vector x (what we want to estimate)
        '''
        For every 1 physical dimension, need 2 state variables, and anything interacting with this vector needs to have same rows/columns as state.
        dim is essentially spatial dimensions (physical world)
        '''
        x = np.zeros(2*dim)
        
        # State Covariance P (Uncertainity in estimate)
        '''
        Multiplying with 500 or 1000: Basically saying no idea where I am starting, 
        saying inital guess is worthless and trust measurement 100% (Forces K = 1)
        If setting P small, filter stays stubborn, thinking sensor is broken, and takes more steps to drift to truth
        Number doesnt matter, as long as its signiicantly larger than sensor noise R
        ''' 
        P = np.eye(2*dim) * 500
        
        # State Transition model F (Physics/Calculated)
        '''
        Here, dt being constant, makes this a Non-Time Variant Kalman Filter. It tells the filter how much "physics" 
        happened since last time it was checked. IRL, sensors are messy and data doesnt come in at a constant rate,
        so simply update it faster.
        '''
        F = np.eye(2*dim)
        for i in range(dim):
            F[i, i+dim] = dt
        
        # Obersvation model H (Maps the state to what the sensor actually measures)
        '''
        For example, here although the state has 2 variables, position and velocity, since the sensor only tells us 
        position, it effectively is used to ignore the velocity part
        '''
        H = np.zeros((dim, 2*dim))
        for j in range(dim):
            H[j, j] = 1
        
        # Process Noise covariance Q (Uncertainity in physics/environment)
        '''
        Q lives in state world (x = [p,v], for 1D). Represents jitters in physics. For example a gust of wind or a bump.
        By setting Q = 0.1: Means object follows law of physics almost perfectly. Heavily relies on F matrix (own prediction).
        Results in straighter lines
        '''
        Q = np.eye(2*dim) * 0.1

        # Measurement Noise covariance R (Uncertainity in Sensor)
        '''
        R lives in Sensor world (z). Represents jitters in Sensor. Since theres only 1 sensor (for position), 
        matches measurement vector (z), if 2 sensors, then in 1D its a 2x2 matrix. Sensor errors are usually uncorrelated
        By setting R = 10: Means sensor is noisy, on average reading can be off by about sqrt(10) = 3.16m
        Filter assumes it to be just noise and doesnt chase immediately, hence a smoother curve
        '''
        R = np.eye(dim) * 10

        '''
        R/Q ratio determines lag:
        IF Sensor is garbage (High R) and physics is perfect (low Q): filter is extremely smooth but slow to react with turns.
        IF Sensor is accurate (Low R) and physics isnt good (High Q): Zero lag but lines look jagged, preserving all noise.
        To tune, record 1000 readings, calculate variance (sigma^2), and use that exact number for R, then play with Q
        '''

        if ActiveControl:
            # Control Input vector u (Example: accelerating)
            u = np.ones(dim) * 1.0  # Result if dim=2: [1.0, 1.0]
            
            # Control Matrix B
            B = np.zeros((2*dim, dim))
            for i in range(dim):
                B[i, i] = 0.5 * dt**2       # Position effect
                B[i + dim, i] = dt          # Velocity effect
        else:
            u = np.zeros(dim)
            B = np.zeros((2*dim, dim))
        '''
        Passive Tracking: u=0, treating any "acceleration" as random noise using Q matrix, thus making the filter lag
        Active Control: Add B*u, predicting movement before sensors measure it, thus lag disappearing
        '''
        Filteredpath = []
        
        for k in range(len(measurements)):
            z = measurements[k]
            # Prediction:
            Xpredicted = F @ x + B @ u
            Ppredicted = F @ P @ F.T + Q

            # Update:
            xinnovation = z - H @ Xpredicted # Innovation (Difference between measured and predicted)
            covinnovation = H @ Ppredicted @ H.T + R  # Covariance Innovation
            K = Ppredicted @ H.T @ np.linalg.inv(covinnovation) # Kalman Gain

            Xupdated = Xpredicted + K @ xinnovation
            Pupdated = (np.eye(2*dim)- K @ H) @ Ppredicted

            x = Xupdated
            P = Pupdated
            Filteredpath.append(Xupdated[:dim])
        
        return np.array(Filteredpath)