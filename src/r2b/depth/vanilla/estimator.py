import cv2
import numpy as np
import os

class StereoProcessor:
    def __init__(self, width=1264, height=800):
        self.w, self.h = width, height
        
        # Load Calibration
        assets_dir = os.path.join(os.path.dirname(__file__), '../assets') # Note: assets is in parent/assets relative to vanilla/
        try:
            self.K = np.load(os.path.join(assets_dir, 'param_K.npy'))
            self.dist = np.load(os.path.join(assets_dir, 'param_dist.npy'))
        except:
             # Default synthetic
            self.K = np.array([[0.3*width, 0, width/2], [0, 0.3*width, height/2], [0,0,1]], dtype=np.float32)
            self.dist = np.zeros(5)

        # Init Undistort Maps (Full Resolution)
        self.new_K, roi = cv2.getOptimalNewCameraMatrix(self.K, self.dist, (width, height), 0, (width, height))
        self.map1, self.map2 = cv2.initUndistortRectifyMap(self.K, self.dist, None, self.new_K, (width, height), cv2.CV_16SC2)

        # Construct Q for DOWNSAMPLED resolution (w/2, h/2) assuming pyrDown
        fx = self.new_K[0,0] / 2.0
        cx = self.new_K[0,2] / 2.0
        cy = self.new_K[1,2] / 2.0

        self.Q = np.float32([
            [1, 0,  0, -cx], 
            [0,-1,  0,  cy],
            [0, 0,  0,  fx],
            [0, 0, -1/0.063, 0] # Baseline in meters
        ])

        self.matcher = cv2.StereoSGBM_create(
            minDisparity=10,
            numDisparities=32, # max_disp (42) - min_disp (10)
            blockSize=5,
            uniquenessRatio=10,
            speckleWindowSize=1000,
            speckleRange=10,
            disp12MaxDiff=25,
            P1=8*3*5**2,
            P2=32*3*5**2
        )
        self.kernel = np.ones((13,13), np.uint8)

    def compute_3d_metrics(self, left, right):
        # 1. Undistort (Full Res) -> Gray -> Downsample
        if left.ndim == 3: left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        if right.ndim == 3: right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

        l_rect = cv2.remap(left, self.map1, self.map2, cv2.INTER_LINEAR)
        r_rect = cv2.remap(right, self.map1, self.map2, cv2.INTER_LINEAR)
        
        l_gray = cv2.pyrDown(l_rect)
        r_gray = cv2.pyrDown(r_rect)
        
        # 2. Disparity computation
        disp = self.matcher.compute(l_gray, r_gray)
        
        # 3. Denoising (Morphology as requested)
        disp_norm = (disp.astype(np.float32)/16.0 - 10) / 32
        disp_vis = (disp_norm - disp_norm.min()) * 255
        denoised = cv2.morphologyEx(disp_vis.astype(np.uint8), cv2.MORPH_CLOSE, self.kernel)
        
        # 4. Reprojection to Meters
        points = cv2.reprojectImageTo3D(disp, self.Q)
        z_values = points[:,:,2].flatten()
        
        # Filter invalid points (safety-first)
        valid_z = z_values[~np.isinf(z_values) & ~np.isnan(z_values) & (z_values > 0)]
        
        if valid_z.size == 0:
            return 0, 0, 0, denoised

        # 5. Statistical Safety Metrics (30% logic)
        indices = valid_z.argsort()
        p = int(valid_z.size * 0.30)
        
        min_dist = np.mean(valid_z[indices[:p]])
        avg_dist = np.mean(valid_z)
        max_dist = np.mean(valid_z[indices[-p:]])
        
        return min_dist, avg_dist, max_dist, denoised