import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from chassis.ugv_driver import UgvDriver
from chassis.ugv_bringup import ugv_bringup


def test_e_stop_overrides_cmd_vel():
    rclpy.init()
    node = UgvDriver("test_driver", test_mode=True)
    msg = Twist()
    msg.linear.x = 0.5
    msg.angular.z = 0.1
    node.cmd_vel_callback(msg)
    assert node.last_velocity_command["X"] == 0.5
    e_stop = Bool()
    e_stop.data = True
    node.e_stop_callback(e_stop)
    node.cmd_vel_callback(msg)
    assert node.last_velocity_command["X"] == 0.0
    node.e_stop_watchdog()
    assert node.last_velocity_command["X"] == 0.0
    e_stop.data = False
    node.e_stop_callback(e_stop)
    node.cmd_vel_callback(msg)
    assert node.last_velocity_command["X"] == 0.5
    node.destroy_node()
    rclpy.shutdown()


def test_min_turn_when_stopped():
    rclpy.init()
    node = UgvDriver("test_driver2", test_mode=True)
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.1
    node.cmd_vel_callback(msg)
    assert abs(node.last_velocity_command["Z"] - 0.2) < 1e-6
    node.destroy_node()
    rclpy.shutdown()


def test_odom_update_from_wheels():
    rclpy.init()
    node = ugv_bringup(test_mode=True)
    now_ns = node.get_clock().now().nanoseconds
    node.last_odom_time_ns = now_ns - int(1e9)
    node.last_left = 0.0
    node.last_right = 0.0
    node.base_controller.base_data = {
        "T": 1001,
        "L": 0,
        "R": 0,
        "ax": 0,
        "ay": 0,
        "az": 0,
        "gx": 0,
        "gy": 0,
        "gz": 0,
        "mx": 0,
        "my": 0,
        "mz": 0,
        "odl": 100,
        "odr": 100,
        "v": 0,
    }
    node.publish_odom_raw()
    assert node.last_odom_raw is not None
    assert node.last_odom_raw.pose.pose.position.x > 0.0
    assert abs(node.last_odom_raw.twist.twist.linear.x - 1.0) < 1e-6
    node.destroy_node()
    rclpy.shutdown()


def test_imu_raw_scaling():
    rclpy.init()
    node = ugv_bringup(test_mode=True)
    node.base_controller.base_data = {
        "T": 1001,
        "L": 0,
        "R": 0,
        "ax": 8192,
        "ay": 0,
        "az": 0,
        "gx": 0,
        "gy": 0,
        "gz": 0,
        "mx": 0,
        "my": 0,
        "mz": 0,
        "odl": 0,
        "odr": 0,
        "v": 0,
    }
    node.publish_imu_data_raw()
    assert node.last_imu_data_raw is not None
    assert abs(node.last_imu_data_raw.linear_acceleration.x - 9.8) < 1e-6
    node.destroy_node()
    rclpy.shutdown()
