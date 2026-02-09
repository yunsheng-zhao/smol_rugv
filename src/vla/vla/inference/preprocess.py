import numpy as np
from typing import Dict, Any

class InputMapper:
    """
    Maps raw ROS data (from SharedBuffer) to the feature dictionary expected by LeRobot.
    """
    def __init__(self):
        # TODO: These keys should ideally be configurable or inferred from the model config
        # "observation.images.camera" is a common convention in LeRobot datasets
        self.image_key = "observation.images.camera" 
        self.state_key = "observation.state"
        self.task_key = "task"

    def map(self, snapshot_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Convert snapshot data (numpy/strings) to the dict structure expected by LeRobot's processor.
        """
        mapped = {}
        
        # 1. Image
        if snapshot_data.get("image") is not None:
            mapped[self.image_key] = snapshot_data["image"]

        # 2. State (Proprioception)
        # We map Odometry to 'observation.state'.
        # Assuming the model expects [vx, vy, wz] for a mobile base
        if snapshot_data.get("odom") is not None:
            odom = snapshot_data["odom"]
            # Construct a state vector. 
            # Current best guess for a UGV: Linear Velocity (x, y) + Angular Velocity (z)
            # This matches typical 'velocity control' inputs.
            # If the model expects absolute position, we would use odom['position'].
            # Using velocity is safer for local control policies.
            state_vec = np.concatenate([
                odom["linear_velocity"][:2],  # vx, vy
                odom["angular_velocity"][2:3] # wz
            ]).astype(np.float32)
            mapped[self.state_key] = state_vec
        
        # 3. Task (Instruction)
        # Ensure it's a string
        mapped[self.task_key] = snapshot_data.get("instruction") or ""

        return mapped
