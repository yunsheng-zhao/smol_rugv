import threading
import numpy as np
from collections import deque
from typing import Optional, Tuple

class ActionQueue:
    """
    Thread-safe queue to store action sequences (chunks) from the VLA model.
    The Control Loop consumes actions from this queue at a fixed frequency.
    """
    def __init__(self, max_len: int = 100):
        self._lock = threading.Lock()
        # We use a deque of (vx, wz) tuples
        self._queue = deque(maxlen=max_len)
        
    def put_chunk(self, actions: np.ndarray):
        """
        Add a chunk of actions to the queue.
        Strategy: Overwrite the remaining queue with the new chunk (Receding Horizon Control).
        
        Args:
            actions: numpy array of shape (N, D), where N is chunk size, D is action dim.
        """
        with self._lock:
            # Clear old actions as the new inference is more up-to-date
            # This implements the "Receding Horizon" strategy common in ACT/VLA
            self._queue.clear()
            
            # Parse actions
            for i in range(actions.shape[0]):
                action = actions[i]
                vx, wz = 0.0, 0.0
                
                if action.shape[0] == 2:
                    vx, wz = float(action[0]), float(action[1])
                elif action.shape[0] == 3:
                    # Assuming [vx, vy, wz]
                    vx, wz = float(action[0]), float(action[2])
                elif action.shape[0] >= 14:
                    # Aloha 14-dim action. 
                    # We need to know which indices correspond to base velocity.
                    # Usually, if trained with base, it's either separate or part of the vector.
                    # For now, we assume we can't control it if it's pure arm, or map first 2 dims.
                    # TODO: Verify index mapping for specific model.
                    pass
                
                self._queue.append((vx, wz))

    def get_next_action(self) -> Optional[Tuple[float, float]]:
        """
        Consume the next action from the queue.
        Returns:
            (vx, wz) tuple or None if queue is empty.
        """
        with self._lock:
            if not self._queue:
                return None
            return self._queue.popleft()
    
    def clear(self):
        with self._lock:
            self._queue.clear()
