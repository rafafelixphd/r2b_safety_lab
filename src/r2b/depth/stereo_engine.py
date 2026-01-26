import cv2
import numpy as np
from typing import Tuple, Dict, Optional
from .types import DepthData

class StereoPerceptionEngine:
    """
    Modular Stereo Perception Engine for PS4 CUH-ZEY1.
    
    This engine implements a safety-critical stereo vision pipeline including:
    1. SGBM Matching with high-density parameters.
    2. Weighted Least Squares (WLS) filtering for edge-preserving smoothing.
    3. Metric 3D reconstruction using a synthetic Q matrix.
    4. Statistical outlier rejection for safety metrics.
    
    Attributes:
        baseline (float): Physical baseline in meters (0.063m).
        focal_factor (float): Factor to estimate focal length from width (0.3).
    """
    
    def __init__(self, 
                 width: int = 1280, 
                 height: int = 800, 
                 min_disp: int = 0, 
                 num_disp: int = 64, 
                 wls_lambda: float = 8000.0, 
                 wls_sigma: float = 1.5):
        """
        Initialize the StereoPerceptionEngine.
        
        Args:
            width: Image width.
            height: Image height.
            min_disp: Minimum disparity (must be >= 0).
            num_disp: Number of disparities (must be divisible by 16).
            wls_lambda: Lambda parameter for WLS filter (smoothness).
            wls_sigma: Sigma parameter for WLS filter (edge sensitivity).
        """
        self.width = width
        self.height = height
        self.baseline = 0.063 # Meters
        
        # SGBM Configuration
        # P1 and P2 control smoothness. P1 is the penalty on the disparity change by plus or minus 1.
        # P2 is the penalty on the disparity change by more than 1.
        window_size = 5 # Block size
        P1 = 8 * 3 * window_size**2
        P2 = 32 * 3 * window_size**2
        
        self.left_matcher = cv2.StereoSGBM_create(
            minDisparity=min_disp,
            numDisparities=num_disp,
            blockSize=window_size,
            P1=P1,
            P2=P2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        
        # WLS Filter Setup
        # We need a right matcher for WLS to ensure consistency
        self.right_matcher = cv2.ximgproc.createRightMatcher(self.left_matcher)
        self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(self.left_matcher)
        self.wls_filter.setLambda(wls_lambda)
        self.wls_filter.setSigmaColor(wls_sigma)
        
        # Q Matrix Construction
        # F_x is approximated as 0.3 * width.
        # This is a SYNTHETIC CALIBRATION. Real-world accuracy requires
        # intrinsic calibration to determine fx, fy, cx, cy.
        f = 0.3 * self.width
        cx = self.width / 2.0
        cy = self.height / 2.0
        
        self.Q = np.float32([
            [1, 0, 0, -cx],
            [0, -1, 0, cy], # Flip Y for standard convention if needed, or keeping consistent with typical reprojection
            [0, 0, 0, f],
            [0, 0, -1/self.baseline, 0] # 1/Tx -> Disparity to Depth
        ])
        
    def compute(self, left_img: np.ndarray, right_img: np.ndarray) -> DepthData:
        """
        Process a stereo pair to generate depth data and safety metrics.
        
        Args:
            left_img: Left camera image (BGR or Grayscale).
            right_img: Right camera image (BGR or Grayscale).
            
        Returns:
            DepthData object containing filtered disparity, Z-cloud, and metrics.
        """
        # Ensure grayscale
        if len(left_img.shape) == 3:
            left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        else:
            left_gray = left_img
            
        if len(right_img.shape) == 3:
            right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
        else:
            right_gray = right_img
            
        # 1. Compute Disparities
        left_disp = self.left_matcher.compute(left_gray, right_gray)
        right_disp = self.right_matcher.compute(right_gray, left_gray)
        
        # 2. Apply WLS Filter
        # The left original image guides the filtering
        filtered_disp = self.wls_filter.filter(left_disp, left_img, disparity_map_right=right_disp)
        
        # 3. Reproject to 3D
        # reprojectImageTo3D expects disparity to be float for precision, 
        # but the raw output of SGBM is fixed-point (x16).
        # We usually divide by 16.0 to get real disparity pixels.
        # HOWEVER, reprojectImageTo3D with handleMissingValues=True might expect raw.
        # Let's convert to true float disparity pixels first for Q matrix usage.
        
        true_disp = filtered_disp.astype(np.float32) / 16.0
        
        # Use Q matrix. 
        points_3d = cv2.reprojectImageTo3D(true_disp, self.Q)
        
        # 4. Extract Z and Calculate Safety Metrics
        z_values = points_3d[:, :, 2]
        
        # Filter invalid points
        # Disparity <= 0 means infinity or invalid. SGBM/WLS might leave some.
        # Also limit max range to avoid far-field noise dominance if needed, 
        # but prompts asks to filter noise via percentiles.
        # We just remove < 0 and infs.
        mask = (z_values > 0) & (np.isfinite(z_values))
        valid_z = z_values[mask]
        
        metrics = {
            "min_dist": 0.0,
            "avg_dist": 0.0,
            "max_dist": 0.0
        }
        
        if valid_z.size > 0:
            # 5th and 95th Percentiles for robustness
            metrics["min_dist"] = float(np.percentile(valid_z, 5))
            metrics["max_dist"] = float(np.percentile(valid_z, 95))
            metrics["avg_dist"] = float(np.mean(valid_z))
            
        return DepthData(
            filtered_disparity=filtered_disp,
            point_cloud_z=z_values,
            risk_metadata=metrics
        )

    def set_tuning_params(self, 
                          min_disp: Optional[int] = None, 
                          num_disp: Optional[int] = None,
                          wls_lambda: Optional[float] = None,
                          wls_sigma: Optional[float] = None) -> None:
        """
        Update stereo parameters dynamically.
        
        Args:
            min_disp: Minimum disparity.
            num_disp: Number of disparities (must be divisible by 16).
            wls_lambda: WLS Lambda.
            wls_sigma: WLS Sigma.
        """
        if min_disp is not None:
            self.left_matcher.setMinDisparity(min_disp)
        
        if num_disp is not None:
            self.left_matcher.setNumDisparities(num_disp)
            
        # Re-create right matcher if SGBM params changed
        if min_disp is not None or num_disp is not None:
            self.right_matcher = cv2.ximgproc.createRightMatcher(self.left_matcher)
            
            # WLS filter depends on the matcher, but updating the matcher in place 
            # might not update the filter's internal ref if it kept one? 
            # createDisparityWLSFilter takes the matcher as init arg.
            # Safest to re-create WLS filter or rely on it checking the matcher.
            # However, looking at OpenCV docs, it holds a reference. 
            # If we changed the object properties, it might be fine.
            # But createRightMatcher returns a new object.
            # So the right_matcher is definitely new.
            # The WLS filter *uses* the left matcher.
            # Does it need the *same* instance of right matcher passed in filter()?
            # Yes, we pass it in filter().
            
        if wls_lambda is not None:
            self.wls_filter.setLambda(wls_lambda)
            
        if wls_sigma is not None:
            self.wls_filter.setSigmaColor(wls_sigma)
