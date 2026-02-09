import threading
from typing import Dict, Any

class SharedBuffer:
    """
    Thread-safe buffer to store the latest observations from ROS topics.
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._data: Dict[str, Any] = {
            "image": None,       # cv2 image (numpy array)
            "odom": None,        # numpy array or dict
            "imu": None,         # numpy array or dict
            "instruction": None  # string
        }
        self._timestamps: Dict[str, float] = {
            "image": 0.0,
            "odom": 0.0,
            "imu": 0.0,
            "instruction": 0.0
        }

    def update(self, key: str, value: Any, timestamp: float):
        """
        Update the buffer with new data.
        """
        with self._lock:
            self._data[key] = value
            self._timestamps[key] = timestamp

    def get_snapshot(self) -> Dict[str, Any]:
        """
        Get a snapshot of the current data and timestamps.
        Returns:
            Dict with 'data' and 'timestamps' keys.
        """
        with self._lock:
            return {
                "data": self._data.copy(),
                "timestamps": self._timestamps.copy()
            }
