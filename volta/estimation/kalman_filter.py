"""Linear Kalman filter."""
import numpy as np


class KalmanFilter:
    def __init__(self, A, B, d, H, Q, R, P, x):
        self.A = A # state transition matrix
        self.B = B # control input matrix
        self.d = d # drift term
        self.H = H # observation matrix
        self.Q = Q # process noise covariance
        self.R = R # measurement noise covariance
        self.P = P # estimate error covariance
        self.x = x # state estimate

    def predict(self, u):
        self.x = self.A @ self.x + self.B @ u + self.d
        self.P = self.A @ self.P @ self.A.T + self.Q
        return self.x, self.P

    def update(self, z):
        # kalman gain
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # update the state estimate
        y = z - self.H @ self.x
        self.x = self.x + K @ y
        
        # update the estimate covariance
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P
        
        return self.x, self.P

def block_diag(*arrays):
    arrays = [np.atleast_2d(a) if np.isscalar(a) else np.atleast_2d(a) for a in arrays]

    rows = sum(arr.shape[0] for arr in arrays)
    cols = sum(arr.shape[1] for arr in arrays)
    block_matrix = np.zeros((rows, cols), dtype=arrays[0].dtype)

    current_row = 0
    current_col = 0

    for arr in arrays:
        r, c = arr.shape
        block_matrix[current_row:current_row + r, current_col:current_col + c] = arr
        current_row += r
        current_col += c

    return block_matrix