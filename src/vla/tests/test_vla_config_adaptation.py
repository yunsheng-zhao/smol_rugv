
import unittest
from unittest.mock import MagicMock, patch
import sys
import os
import logging

# Add src/vla to path to import vla package
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

# Mock modules that might not be present or we want to control
sys.modules['torch'] = MagicMock()
sys.modules['lerobot'] = MagicMock()
sys.modules['lerobot.policies.smolvla.modeling_smolvla'] = MagicMock()
sys.modules['lerobot.policies.factory'] = MagicMock()

from vla.model.smol_vla_policy import SmolVLAPolicyWrapper

class TestSmolVLAConfigAdaptation(unittest.TestCase):
    def setUp(self):
        # Setup Logger mock to capture warnings
        self.logger_patch = patch('vla.model.smol_vla_policy.logging.getLogger')
        self.mock_get_logger = self.logger_patch.start()
        self.mock_logger = MagicMock()
        self.mock_get_logger.return_value = self.mock_logger

    def tearDown(self):
        self.logger_patch.stop()

    @patch('vla.model.smol_vla_policy.SmolVLAPolicy')
    @patch('vla.model.smol_vla_policy.make_pre_post_processors')
    @patch('vla.model.smol_vla_policy.torch')
    def test_force_disable_aloha_adaptation(self, mock_torch, mock_make_processors, mock_policy_cls):
        """
        Purpose: Verify that adapt_to_pi_aloha is forcibly set to False for UGV.
        Reason: Aloha adaptations (joint flipping) corrupt UGV velocity controls.
        """
        # Setup Mock Policy and Config
        mock_policy_instance = MagicMock()
        mock_config = MagicMock()
        mock_config.adapt_to_pi_aloha = True  # Simulate a config that has this enabled
        mock_config.max_action_dim = 2        # Correct dimensions
        mock_policy_instance.config = mock_config
        
        mock_policy_cls.from_pretrained.return_value = mock_policy_instance
        
        # Setup mock make_pre_post_processors to return two values
        mock_make_processors.return_value = (MagicMock(), MagicMock())
        
        # Setup Mock Torch device
        mock_torch.cuda.is_available.return_value = False
        mock_torch.device.return_value = MagicMock(type='cpu')

        # Initialize Wrapper
        wrapper = SmolVLAPolicyWrapper(model_id="mock_model", device="cpu")

        # Assertion
        self.assertFalse(mock_config.adapt_to_pi_aloha, 
                         "adapt_to_pi_aloha should be set to False by the wrapper")
        self.mock_logger.warning.assert_any_call(
            "Disabling 'adapt_to_pi_aloha' in config to prevent invalid action transformation for UGV."
        )

    @patch('vla.model.smol_vla_policy.SmolVLAPolicy')
    @patch('vla.model.smol_vla_policy.make_pre_post_processors')
    @patch('vla.model.smol_vla_policy.torch')
    def test_action_dim_mismatch_warning(self, mock_torch, mock_make_processors, mock_policy_cls):
        """
        Purpose: Verify that a warning is logged if the model action dimension is not 2.
        Reason: Helps developers identify if they loaded the wrong model (e.g. 14-dim arm model).
        """
        # Setup Mock Policy and Config
        mock_policy_instance = MagicMock()
        mock_config = MagicMock()
        mock_config.adapt_to_pi_aloha = False
        mock_config.max_action_dim = 14       # Wrong dimensions (e.g. original Aloha)
        mock_policy_instance.config = mock_config
        
        mock_policy_cls.from_pretrained.return_value = mock_policy_instance
        
        # Setup mock make_pre_post_processors to return two values
        mock_make_processors.return_value = (MagicMock(), MagicMock())
        
        # Setup Mock Torch
        mock_torch.cuda.is_available.return_value = False
        mock_torch.device.return_value = MagicMock(type='cpu')

        # Initialize Wrapper
        wrapper = SmolVLAPolicyWrapper(model_id="mock_model", device="cpu")

        # Assertion
        # Check if warning was logged with specific content
        expected_warning_fragment = "Model action dimension mismatch! Expected 2 (v, w), but got 14."
        
        # Find if any call args contained the fragment
        found = False
        for call in self.mock_logger.warning.call_args_list:
            if expected_warning_fragment in call[0][0]:
                found = True
                break
        
        self.assertTrue(found, f"Warning about dimension mismatch not found. Logs: {self.mock_logger.warning.call_args_list}")

if __name__ == '__main__':
    unittest.main()
