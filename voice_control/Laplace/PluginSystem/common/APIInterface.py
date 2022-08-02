import logging
import sys
import rclpy.logging
class APIInterface(object):
    def __init__(self, *args, **kwargs):
        self.LogTool = logging.getLogger()