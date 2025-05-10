# smoothing.py
import numpy as np

class EstimatorPredictor:
    def __init__(self, alpha=0.5, initial_angle=90.0):
        """
        Complementary filter for angle estimation.
        
        Parameters:
        - alpha: Weighting factor for the gyroscope integration (typically close to 1, e.g., 0.95-0.98).
                 A higher alpha favors the gyroscope data, while (1 - alpha) favors the accelerometer.
        - initial_angle: Starting angle estimate.
        """
        self.alpha = alpha
        self.angle = initial_angle

    def update(self, gyro_rate, accel_angle, dt):
        """
        Update the angle estimate using the complementary filter.
        
        Parameters:
        - gyro_rate: Angular velocity from the gyroscope (in degrees/second or radians/second).
        - accel_angle: Angle computed from the accelerometer measurements.
        - dt: Time step duration since the last update.
        
        Steps:
        1. Gyroscope Integration (Prediction):
           Compute the predicted angle by integrating the gyroscope rate over the time step:
           gyro_angle = previous angle + gyro_rate * dt.
        
        2. Complementary Filter Blend:
           Combine the predicted angle with the accelerometer angle:
           angle = alpha * gyro_angle + (1 - alpha) * accel_angle.
        
        Returns:
        - The updated angle estimate.
        """
        # Step 1: Integrate the gyroscope reading
        gyro_angle = self.angle + gyro_rate * dt
        
        # Step 2: Blend with the accelerometer angle using the complementary filter formula
        self.angle = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle
        
        return self.angle

class MovingAverageFilter:
    def __init__(self, window_size=5):
        """
        Moving Average Filter for smoothing data.
        
        Parameters:
        - window_size: The number of recent values to average.
        """
        self.window_size = window_size
        self.values = []

    def update(self, value):
        """
        Update the filter with a new value and return the current moving average.
        
        Parameters:
        - value: New data point to include in the average.
        
        Returns:
        - The average of the last 'window_size' values.
        """
        self.values.append(value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
        return np.mean(self.values)
    
class KalmanFilter:
    def __init__(self, initial_state=0.0, process_variance=1e-5, measurement_variance=0.1):
        """
        A simple 1D Kalman filter for estimating a state (e.g., angle) from noisy measurements.
        
        Parameters:
        - initial_state: The initial estimate of the state.
        - process_variance (Q): Variance in the process model (how much we trust the model).
        - measurement_variance (R): Variance in the measurements (how noisy the sensor is).
        """
        self.x = initial_state   # State estimate
        self.P = 1.0             # Estimation error covariance
        self.Q = process_variance  # Process noise covariance
        self.R = measurement_variance  # Measurement noise covariance

    def update(self, measurement):
        """
        Update the state estimate using a new measurement.
        
        Steps:
        1. Prediction Update:
           Increase the estimation error covariance by the process noise.
           P = P + Q.
        
        2. Measurement Update:
           Compute the Kalman gain:
           K = P / (P + R).
           Update the state estimate with the new measurement:
           x = x + K * (measurement - x).
           Update the error covariance:
           P = (1 - K) * P.
        
        Parameters:
        - measurement: The new measurement from the sensor.
        
        Returns:
        - The updated state estimate.
        """
        # Prediction update: Increase the error covariance by the process noise
        self.P = self.P + self.Q
        
        # Measurement update: Compute the Kalman gain
        K = self.P / (self.P + self.R)
        
        # Update the state estimate with the measurement
        self.x = self.x + K * (measurement - self.x)
        
        # Update the error covariance
        self.P = (1 - K) * self.P
        
        return self.x
