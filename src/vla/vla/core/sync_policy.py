import time
from typing import Dict, Any
import logging

class SyncPolicy:
    """
    Policy to determine if the current snapshot of data is valid for inference.
    """
    def __init__(self, image_timeout: float = 0.5, odom_timeout: float = 0.2):
        self.image_timeout = image_timeout
        self.odom_timeout = odom_timeout
        self.logger = logging.getLogger("SyncPolicy")

    def is_valid(self, snapshot: Dict[str, Any], current_time: float) -> bool:
        """
        Check if the snapshot has fresh and valid data.
        Args:
            snapshot: Data snapshot from buffer
            current_time: Current system/ROS time in seconds (float)
        """
        data = snapshot["data"]
        timestamps = snapshot["timestamps"]
        
        # 1. Check Image existence and freshness
        if data["image"] is None:
            # self.logger.warning("No image data available.")
            return False
            
        if current_time - timestamps["image"] > self.image_timeout:
            # self.logger.warning(f"Image data is stale. Delay: {current_time - timestamps['image']:.3f}s")
            return False

        # 2. Check Odom existence (optional, depending on whether model strictly needs it)
        # For SmolVLA, proprioception is usually required.
        if data["odom"] is None:
            return False
            
        # Odom is high frequency, should be very fresh
        if current_time - timestamps["odom"] > self.odom_timeout:
            return False

        return True
