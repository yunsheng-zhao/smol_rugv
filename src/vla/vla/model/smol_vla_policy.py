import sys
import os
import torch
import logging
from typing import Tuple, Dict, Any

# Path hacking to include lerobot
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
# Attempt to find 'ref_code' by traversing up or using environment variable
# Priority: ENV > Relative Path
LEROBOT_SRC = os.environ.get("LEROBOT_SRC")

if not LEROBOT_SRC:
    # Fallback to relative path assuming standard workspace layout: src/vla/vla/model -> ... -> ref_code
    # model -> vla -> vla -> src -> my_ugv_root -> ref_code
    # This assumes the package is installed in a way that preserves relative structure to ref_code, 
    # which is true for symlink install but fragile otherwise.
    # A more robust way in production is to install lerobot as a python package.
    PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "../../../../.."))
    LEROBOT_SRC = os.path.join(PROJECT_ROOT, "ref_code", "lerobot-main (SmolVLA)", "src")

if os.path.exists(LEROBOT_SRC) and LEROBOT_SRC not in sys.path:
    sys.path.append(LEROBOT_SRC)
else:
    logging.warning(f"LeRobot source not found at {LEROBOT_SRC}. Ensure 'ref_code' exists or set LEROBOT_SRC env var.")

try:
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
    from lerobot.policies.factory import make_pre_post_processors
except ImportError as e:
    logging.warning(f"Could not import lerobot: {e}. Ensure ref_code is present and dependencies are installed.")
    SmolVLAPolicy = None

class SmolVLAPolicyWrapper:
    """
    Wrapper for the SmolVLA policy from LeRobot.
    Handles model loading, FP16 conversion, and processor initialization.
    """
    def __init__(self, model_id: str, device: str = "cuda"):
        if SmolVLAPolicy is None:
            raise ImportError("LeRobot library not found or failed to import.")

        self.logger = logging.getLogger("SmolVLAPolicy")
        
        # Check if cuda is actually available
        if device == "cuda" and not torch.cuda.is_available():
            self.logger.warning("CUDA not available, falling back to CPU.")
            device = "cpu"
            
        self.device = torch.device(device)
        
        self.logger.info(f"Loading SmolVLA model: {model_id} on {self.device}...")
        
        try:
            # Load pretrained policy
            self.policy = SmolVLAPolicy.from_pretrained(model_id)
            self.policy.to(self.device)
            
            # --- VLA Adaptation for UGV ---
            # 1. Force disable Aloha-specific adaptations (joint flipping/gripper conversion)
            #    This is critical because our action space is [v, w], not mechanical arm joints.
            if hasattr(self.policy.config, 'adapt_to_pi_aloha') and self.policy.config.adapt_to_pi_aloha:
                self.logger.warning("Disabling 'adapt_to_pi_aloha' in config to prevent invalid action transformation for UGV.")
                self.policy.config.adapt_to_pi_aloha = False
                
            # 2. Check Action Dimension
            #    We expect 2 dimensions (v, w). If the loaded model has more (e.g., original 14D model),
            #    we log a warning but proceed (the inference loop will handle slicing).
            expected_action_dim = 2
            if self.policy.config.max_action_dim != expected_action_dim:
                self.logger.warning(
                    f"Model action dimension mismatch! Expected {expected_action_dim} (v, w), "
                    f"but got {self.policy.config.max_action_dim}. "
                    "Ensure you are using the fine-tuned UGV model. "
                    "Inference will proceed but actions may be sliced."
                )
            # -----------------------------

            # FP16 Optimization for CUDA
            if self.device.type == 'cuda':
                self.logger.info("Converting model to FP16 for optimization.")
                self.policy.half()
            
            self.policy.eval()
            
            # Initialize processors
            self.logger.info("Initializing pre/post processors...")
            self.preprocess_pipeline, self.postprocess_pipeline = make_pre_post_processors(
                self.policy.config,
                pretrained_name_or_path=model_id,
                preprocessor_overrides={"device_processor": {"device": str(self.device)}}
            )
            
            self.logger.info("Model loaded successfully.")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize SmolVLA: {e}")
            raise

    def preprocess(self, observation: Dict[str, Any]) -> Dict[str, Any]:
        """
        Run the preprocessing pipeline.
        Args:
            observation: Dict of numpy arrays (raw data from ROS).
        Returns:
            Dict of torch tensors on device.
        """
        # The pipeline expects a specific structure. 
        # Typically keys like 'observation.images.camera1', 'observation.state', etc.
        # This mapping should be handled before calling this, or the observation dict passed here
        # must already match what the processor expects.
        return self.preprocess_pipeline(observation)

    def step(self, batch: Dict[str, Any]) -> torch.Tensor:
        """
        Run inference.
        Args:
            batch: Preprocessed batch.
        Returns:
            Action tensor (normalized).
        """
        with torch.no_grad():
            action = self.policy.select_action(batch)
            return action

    def postprocess(self, action: torch.Tensor) -> torch.Tensor:
        """
        Unnormalize the action.
        """
        return self.postprocess_pipeline(action)
