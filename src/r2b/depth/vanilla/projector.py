import cv2
import numpy as np


class RiskAwareProjector:
    def __init__(self, Q_matrix):
        self.Q = Q_matrix

    def get_safe_distance(self, disparity):
        # Reproject to 3D space
        points_3d = cv2.reprojectImageTo3D(disparity, self.Q)
        
        # Extract Z (depth) and filter out non-positive/infinite values
        z_vals = points_3d[:, :, 2]
        mask = (z_vals > 0) & (z_vals < 10.0) # Assume 10m max range for safety
        
        valid_z = z_vals[mask]
        
        if valid_z.size == 0:
            return float('inf')

        # Safety Logic: Use the 5th percentile for "Minimum Distance" 
        # to avoid being fooled by a single noisy pixel.
        return np.percentile(valid_z, 5)