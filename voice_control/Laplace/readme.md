Laplace 是一个插件化实现的语音助手

# 下载
直接克隆当前仓库到您的项目之中
```shell
git clone https://github.com/AnthonySun256/Laplace.git
```

# 使用
将 Laplace 文件夹放到您的项目之中，并新建一个 Plugins 文件夹存放插件：具体可以参考 examples 文件夹

新建插件方法如下：
首先在 Plugins 文件夹中新建 文件夹并在其中新建名为 Plugin.py 的文件。这样 插件名即为 文件夹名，例如：
```
Plugins
├─Plugin_test   <----名为 Plugin_test 的插件
│      Plugin.py
│      __init__.py
│
└─Plugin_test2  <----名为 Plugin_test2 的插件>
        Plugin.py
```

``Plugin.py`` 中根据例子填写代码

```python
class Plugin(object): # 插件类
    plugin_version = "20220701" # 插件版本
    supported_api_version = "0.1"   # 插件所需 API 版本（由 PluginManager 提供）
    register_events = ["load", "command"] # 插件要注册的事件，当外部触发相应事件时，会调用当前插件

    def __init__(self, api_version, api_interface): # 获取 API 接口和版本，API 接口用于 使用外部提供的一些功能
        self._api_version = api_version
        self._api_interface = api_interface

    def callback(self, event_type=None, event_value=None): # 插件回调函数，当外部事件触发调用插件时会调用这个函数
        self._api_interface.LogTool.debug(
            f"Recieve event [{event_type}], value [{event_value}]")
            
        if event_type == "load":
            print("Hello World")
        elif event_type == "command":
            if event_value == "forward":
                print("forward")

            elif event_value == "back":
                print("back")

            elif event_value == "left":
                print("left")

            elif event_value == "right":
                print("right")
            elif event_value == "up":
                print("up")

    def unload(self, *args, **kwargs):  # 暂未使用
        pass

    @classmethod
    def capabilities(cls, api_versions: 'list[str]') -> dict: # 插件信息查询接口，查询当前 API 版本是否满足插件需求，满足则返回插件信息

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


def get_class(*args, **kwargs) -> Plugin:   # 初始化时会调用这个函数返回 插件类(注意不是插件实例，因为可能会存在同一个插件需要多个实例的情况)
    return Plugin

```


之后按照如下代码加载 laplace 核心程序
```python
import os
import laplace
ll = laplace.Laplace(Plugin_path=os.path.dirname(__file__)+"/examples/Plugins")
ll.spin()
```
即可开始运行插件

如果您的程序不希望被 ``ll.spin()`` 阻塞，则可以使用 ``ll.spin_once()`` 代替。
``ll.spin_once()`` 只会进行一次事件轮询
