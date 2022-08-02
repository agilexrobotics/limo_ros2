class Plugin(object):
    plugin_version = "20220701"
    supported_api_version = "0.1"
    register_events = ["load"]

    def __init__(self, api_version, api_interface):
        self._api_version = api_version
        self._api_interface = api_interface

    def callback(self, event_type=None, event_value=None):
        if event_type == "load":
            print("Service loaded")

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


# Get the class, not object instance!
def get_class(*args, **kwargs) -> Plugin:
    return Plugin
