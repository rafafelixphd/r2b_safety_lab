from dataclasses import dataclass

@dataclass
class StereoConfig:
    min_disp: int = 10
    num_disp: int = 48
    block_size: int = 5
    lambda_wls: int = 8000
    sigma_wls: float = 1.5