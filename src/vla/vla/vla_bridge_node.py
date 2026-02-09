import rclpy
from rclpy.node import Node
import sys
import logging

from vla.io.ros_io import ROSIO
from vla.core.shared_buffer import SharedBuffer
from vla.core.action_queue import ActionQueue
from vla.inference.vla_loop import VLALoop

class VLABridgeNode(Node):
    def __init__(self):
        super().__init__('vla_bridge_node')
        
        # Parameters
        self.declare_parameter('model_id', 'lerobot/smolvla_base')
        self.declare_parameter('inference_rate', 10.0) # Frequency of Model Inference
        self.declare_parameter('control_rate', 20.0)   # Frequency of Action Execution
        
        model_id = self.get_parameter('model_id').value
        inference_rate = self.get_parameter('inference_rate').value
        control_rate = self.get_parameter('control_rate').value
        
        self.get_logger().info(f"Starting VLABridgeNode with model {model_id}...")
        
        # Initialize components
        self.buffer = SharedBuffer()
        self.action_queue = ActionQueue(max_len=100)
        self.ros_io = ROSIO(self, self.buffer)
        
        # Start Inference Thread
        try:
            self.vla_loop = VLALoop(
                buffer=self.buffer,
                action_queue=self.action_queue,
                io=self.ros_io,
                model_id=model_id,
                frequency=inference_rate
            )
            self.vla_loop.start()
        except Exception as e:
            self.get_logger().fatal(f"Failed to start VLA Loop: {e}")
            # We don't exit strictly here to allow ROS spin to clean up, 
            # but in production this should trigger a restart.
            raise e
            
        # Start Control Timer
        self.create_timer(1.0 / control_rate, self._control_loop)
        
    def _control_loop(self):
        """
        Consumes actions from the queue and publishes them.
        """
        action = self.action_queue.get_next_action()
        if action:
            vx, wz = action
            self.ros_io.publish_cmd_vel(vx, wz)
        else:
            # If queue is empty (inference too slow or stopped), stop the robot for safety
            self.ros_io.publish_cmd_vel(0.0, 0.0)

    def destroy_node(self):
        self.get_logger().info("Stopping VLA Loop...")
        if hasattr(self, 'vla_loop'):
            self.vla_loop.stop()
        super().destroy_node()

def main(args=None):
    # Configure generic logging
    logging.basicConfig(level=logging.INFO)
    
    rclpy.init(args=args)
    
    try:
        node = VLABridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node exited with error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
