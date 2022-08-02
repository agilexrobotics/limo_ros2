from Laplace.laplace import Laplace
import os
import rclpy
from rclpy.node import Node
from threading import Thread
from loguru import logger

class VoiceCommand(Node):
    def __init__(self, name="voice_command"):
        super().__init__(name)
        self.declare_parameter(
            "plugin_path", "/workspaces/limo_workspace/install/voice_control/share/Plugins")

        self.declare_parameter("hot_word", None)
        self.declare_parameter("authentic_key", None)

        self.declare_parameter("mic_index", 11)
        self.declare_parameter("speaker_index", 11)
        self.declare_parameter("recognition_engine", "pocketsphinx")
        self.LogTool = logger

    def setup(self):
        _plugin_path = self.get_parameter("plugin_path").get_parameter_value().string_value

        _hot_word = self.get_parameter("hot_word").get_parameter_value().string_value
        _hot_word = _hot_word if _hot_word != "" else None

        _authentic_key = self.get_parameter("authentic_key").get_parameter_value().string_value
        _authentic_key = _authentic_key if _authentic_key != "" else None

        _mic_index = self.get_parameter("mic_index").get_parameter_value().integer_value
        _speaker_index = self.get_parameter("speaker_index").get_parameter_value().integer_value
        _recognition_engine = self.get_parameter(
            "recognition_engine").get_parameter_value().string_value
        
        self._ll = Laplace(Plugin_path=_plugin_path, hot_word=_hot_word,
                           Authentic_key=_authentic_key, mic_index=_mic_index,
                           speaker_index=_speaker_index, recognition_engine=_recognition_engine,api_interface=self)
        self._ll_spin = Thread(target=self._ll.spin)
        self._ll_spin.daemon = True
        self._ll_spin.start()
        self.get_logger().info("Laplace service started!")

def main(*args, **kwargs):
    rclpy.init(*args, **kwargs)
    n = VoiceCommand()
    n.setup()
    rclpy.spin(n)
    rclpy.shutdown()
