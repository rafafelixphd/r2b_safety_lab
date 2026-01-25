import cv2
import numpy as np

class StereoProcessor:
    def __init__(self, width=1264, height=800):
        self.w, self.h = width, height
        f = 0.3 * self.w
        self.Q = np.float32([
            [1, 0,  0, -0.5 * (self.w // 2)], # Adjusted for downsampling later
            [0,-1,  0,  0.5 * (self.h // 2)],
            [0, 0,  0,  f],
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
        # 1. Downsample for speed (as per your original script)
        l_gray = cv2.pyrDown(cv2.cvtColor(left, cv2.COLOR_BGR2GRAY))
        r_gray = cv2.pyrDown(cv2.cvtColor(right, cv2.COLOR_BGR2GRAY))
        
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