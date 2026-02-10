import threading
import time
import torch
import logging
import numpy as np
from typing import Any

from vla.core.shared_buffer import SharedBuffer
from vla.core.sync_policy import SyncPolicy
from vla.core.action_queue import ActionQueue
from vla.model.smol_vla_policy import SmolVLAPolicyWrapper
from vla.inference.preprocess import InputMapper

class VLALoop(threading.Thread):
    """
    Main inference loop running in a separate thread.
    Fetches data from buffer -> Preprocesses -> Runs Model -> Pushes Action Chunk to Queue.
    """
    def __init__(self, 
                 buffer: SharedBuffer, 
                 action_queue: ActionQueue,
                 io: Any, # ROSIO type
                 model_id: str,
                 frequency: float = 10.0):
        super().__init__()
        self.buffer = buffer
        self.action_queue = action_queue
        self.io = io
        self.frequency = frequency
        self.running = False
        self.logger = logging.getLogger("VLALoop")
        
        # Initialize components
        self.sync_policy = SyncPolicy()
        self.input_mapper = InputMapper()
        
        self.logger.info("Initializing VLA Model (this may take time)...")
        self.model = SmolVLAPolicyWrapper(model_id)
        
        # State flags
        self.has_warned_action_dim = False 
        
    def run(self):
        self.running = True
        self.logger.info("VLA Inference Loop started.")
        
        period = 1.0 / self.frequency
        
        while self.running:
            start_time = time.time()
            
            try:
                self._step()
            except Exception as e:
                self.logger.error(f"Error in inference loop: {e}")
                
            elapsed = time.time() - start_time
            sleep_time = max(0, period - elapsed)
            time.sleep(sleep_time)

    def _step(self):
        # 1. Get data
        snapshot = self.buffer.get_snapshot()
        
        # 2. Validate
        # Use ROS time for sync check
        current_time = self.io.node.get_clock().now().nanoseconds * 1e-9
        if not self.sync_policy.is_valid(snapshot, current_time):
            # Data not fresh enough, skip inference
            return

        # 3. Map inputs
        features = self.input_mapper.map(snapshot["data"])
        
        # 4. Preprocess (to Tensor)
        features_tensor = self.model.preprocess(features)
        
        # 5. Inference
        action_tensor = self.model.step(features_tensor)
        
        # 6. Postprocess
        action_numpy = self.model.postprocess(action_tensor)
        
        # 7. Push to Queue
        self._push_to_queue(action_numpy)

    def _push_to_queue(self, action: Any):
        # Convert to CPU numpy if needed
        if isinstance(action, torch.Tensor):
            action = action.detach().cpu().numpy()
            
        # Handle batch dim (batch_size=1)
        if action.ndim == 3: # (B, T, D)
            action = action[0] 
            
        # action is now (T, D)
        # Verify it's 2D
        if action.ndim != 2:
            self.logger.warning(f"Unexpected action shape: {action.shape}, expected (T, D).")
            return
            
        # Adapt Action Space for UGV
        # We expect D=2 (vx, wz).
        # If D > 2 (e.g., using original 14D model), we slice the first 2 dimensions.
        # This allows us to test the pipeline even with mismatched models.
        if action.shape[1] > 2:
            if not self.has_warned_action_dim:
                self.logger.warning(f"Action dimension is {action.shape[1]}, slicing to first 2 for UGV (vx, wz).")
                self.has_warned_action_dim = True
            action = action[:, :2]
        elif action.shape[1] < 2:
            self.logger.error(f"Action dimension {action.shape[1]} is too small for UGV (needs 2: vx, wz).")
            return

        self.action_queue.put_chunk(action)

    def stop(self):
        self.running = False
        self.join()
