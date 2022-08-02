import os
import sys
import importlib
from .APIInterface import APIInterface


class PluginManager(object):
    """
    A simple plugin manager which support basic plugin discover, load, register...
    TODO: support hot plug
    TODO: support Plugin priority level
    TODO: allow plugins block events
    """
    provide_api_version = ["0.1", "0.2"]

    def __init__(self,api_interface=None) -> None:
        super().__init__()
        self.api = self.create_api_interface() if api_interface is None else api_interface
        self.__plugins = {}

    @classmethod
    def create_api_interface(cls, *args, **kwargs) -> APIInterface:
        """Create ApiInterface
        """
        return APIInterface(*args, **kwargs)

    def _find_plugins(self, plugin_path: str):
        """
        Find plugins from the given path.

        :param plugin_path: The path to the plugin
        """
        plugin_path = os.path.abspath(plugin_path)
        if plugin_path not in sys.path:
            sys.path.append(plugin_path)
        if plugin_path is None:
            self.api.LogTool.critical("Missing arg: [plugin_path]")
            raise RuntimeError("Missing arg plugin_path")

        self.api.LogTool.debug("Now in the path [%s]" % os.getcwd())

        for dirname in os.listdir(plugin_path):
            f = os.path.join(plugin_path, dirname, "Plugin.py")

            self.api.LogTool.debug(
                "Find  plugins [%s] in [%s]" % (os.path.basename(dirname), f))

            if os.path.exists(f):
                _plugin_name = dirname

                if _plugin_name in self.__plugins:
                    self.api.LogTool.warning(
                        f"Plugin [{_plugin_name}] already exists, skipping")
                else:
                    self.__plugins[_plugin_name] = {}
                    self.__plugins[_plugin_name]["path"] = plugin_path
                    self.api.LogTool.info(
                        f"Find plugin [{_plugin_name}]")

    def _load_plugin(self, name):
        _dir_backup = os.getcwd()
        if name in self.__plugins:

            _mod = importlib.import_module(f"{name}.Plugin")

            try:
                _cls = _mod.get_class()
                _plugin_info = _cls.capabilities(self.provide_api_version)
                self.__plugins[name]["module"] = _cls
                self.__plugins[name]["info"] = _plugin_info
                self.api.LogTool.info(f"Plugin [{name}] Load successful")
            except RuntimeError:
                self.api.LogTool.warning(
                    f"Incompatible plugin detected [{name}]")
        os.chdir(_dir_backup)

    def get_all_plugin_name(self):
        return self.__plugins.keys()

    def remove_plugin(self, name):
        if name in sys.modules:
            sys.modules.pop(name)
            self.api.LogTool.info(f"Unload plugin [{name}]")
            self.__plugins[name] = {}
        self.api.LogTool.warning(f"Plugin [{name}] does not be loaded yet")

    def get_plugin_instance(self, name) -> object:
        _instant = None
        if name in self.__plugins:
            _api_version = self.__plugins[name]["info"]["api_version"]
            _instant = self.__plugins[name]["module"](_api_version, self.api)
            self.api.LogTool.debug(f"Create a instance of Plugin [{name}]")

        else:
            self.api.LogTool.warning(
                f"Failed to create plugin instance, because plugin [{name}] does not exist.")

        return _instant

    def get_plugin_events(self, name) -> list:
        _events = []
        if name in self.__plugins:
            _events = self.__plugins[name]["info"]["events"]
        else:
            self.api.LogTool.critical(f"Failed to load info of [{name}]")
        return _events

    def load_plugins(self, path="./PluginSystem/Plugins"):
        self._find_plugins(path)
        for name in self.__plugins.keys():
            self._load_plugin(name)
