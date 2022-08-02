from geometry_msgs.msg import Twist
from rclpy.node import Node, Publisher

class Plugin(object):
    plugin_version = "20220701"
    supported_api_version = "0.1"
    register_events = ["load", "command"]

    def __init__(self, api_version, api_interface):
        self._api_version = api_version
        self._api_interface:Node = api_interface
        self._cmd_vel_msg = Twist()

        self._stop()
    
    def _stop(self):
        self._cmd_vel_msg.angular.x=0.0
        self._cmd_vel_msg.angular.y=0.0
        self._cmd_vel_msg.angular.z=0.0

        self._cmd_vel_msg.linear.x=0.0
        self._cmd_vel_msg.linear.y=0.0
        self._cmd_vel_msg.linear.z=0.0

    def _pub_cmd_vel(self):
        self._cmd_vel_puber.publish(self._cmd_vel_msg)

    def callback(self, event_type=None, event_value=None):
        self._api_interface.LogTool.debug(
            f"Receive event [{event_type}], value [{event_value}]")
            
        if event_type == "load":
            self._cmd_vel_puber = self._api_interface.create_publisher(Twist,"cmd_vel", 10)
            self._timer=self._api_interface.create_timer(0.3, self._pub_cmd_vel)
            self._api_interface.LogTool.info("Plugin_test2 loaded successful!")
        elif event_type == "command":
            self._stop()
            if event_value == "FORWARD":    
                self._cmd_vel_msg.linear.x=0.3
                self._api_interface.LogToll.info("forward")

            elif event_value == "BACK":
                self._cmd_vel_msg.linear.x=-0.3
                self._api_interface.LogToll.info("back")

            elif event_value == "LEFT":
                self._api_interface.LogToll.info("left")

            elif event_value == "RIGHT":
                self._api_interface.LogToll.info("right")
            elif event_value == "STOP":
                self._api_interface.LogToll.info("up")
            else:
                self._api_interface.LogToll.info("unknown command")

    def unload(self, *args, **kwargs):
        pass

    @classmethod
    def capabilities(cls, api_versions: 'list[str]') -> dict:

        if cls.supported_api_version in api_versions:
            _info = {
                "plugin_version": cls.plugin_version,
                "api_version": cls.supported_api_version,
                "events": cls.register_events
            }
            return _info
        else:
            raise RuntimeError(
                f"API versions [{cls.supported_api_version}] are not supported!")


def get_class(*args, **kwargs) -> Plugin:
    return Plugin
