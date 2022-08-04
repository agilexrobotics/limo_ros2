from loguru import logger
class APIInterface(object):
    def __init__(self, *args, **kwargs):
        self.LogTool = logger