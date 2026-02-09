import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import threading

class SpeechNode(Node):
    def __init__(self):
        super().__init__('speech_node')
        self.publisher_ = self.create_publisher(
            String, 
            '/instruction_text', 
            10
        )
        self.get_logger().info('Speech Node has been started (Local Mic Mode).')
        
        # 初始化语音识别器
        self.recognizer = sr.Recognizer()
        
        # 启动监听线程，避免阻塞 ROS 循环
        self.listen_thread = threading.Thread(target=self.listen_loop)
        self.listen_thread.daemon = True
        self.listen_thread.start()

    def listen_loop(self):
        """
        持续监听麦克风输入并进行识别
        """
        while rclpy.ok():
            try:
                # 使用系统默认麦克风
                with sr.Microphone() as source:
                    self.get_logger().info("Listening... (Speak now)")
                    
                    # 动态调整环境噪音阈值
                    self.recognizer.adjust_for_ambient_noise(source, duration=1.0)
                    
                    # 监听音频，设置超时和短语限制
                    audio = self.recognizer.listen(source, timeout=5.0, phrase_time_limit=10.0)
                    
                    self.get_logger().info("Recognizing...")
                    
                    # 使用 PocketSphinx 进行离线识别 (无需联网)
                    try:
                        text = self.recognizer.recognize_sphinx(audio)
                        if text:
                            self.get_logger().info(f"Recognized: '{text}'")
                            self.publish_instruction(text)
                    except sr.UnknownValueError:
                        self.get_logger().warn("Could not understand audio")
                    except sr.RequestError as e:
                        self.get_logger().error(f"Sphinx error: {e}")
                        
            except sr.WaitTimeoutError:
                # 超时未检测到语音，继续下一次循环
                pass
            except Exception as e:
                self.get_logger().error(f"Microphone error: {e}")
                # 等待一会儿再重试，避免死循环刷屏
                import time
                time.sleep(1.0)

    def publish_instruction(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
