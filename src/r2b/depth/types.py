from dataclasses import dataclass
import numpy as np
from typing import Dict

@dataclass
class DepthData:
    """
    Structured output from the StereoPerceptionEngine.
    
    Attributes:
        filtered_disparity (np.ndarray): The WLS-filtered disparity map.
        point_cloud_z (np.ndarray): The Z-coordinate (depth) map in meters.
        risk_metadata (Dict[str, float]): Safety metrics including min (5th percentile), 
                                          avg, and max (95th percentile) distances.
    """
    filtered_disparity: np.ndarray
    point_cloud_z: np.ndarray
    risk_metadata: Dict[str, float]
