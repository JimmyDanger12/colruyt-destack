#!usr/bin/env python

"""
This script will start the process of unloading a beer crate container.
"""

from robot_mechanics.handler import Handler
from globalconfig import GlobalConfig

DEFAULT_CONFIG_PATH = "config.ini"

if __name__ == "__main__":
    handler = Handler()
    config = GlobalConfig(DEFAULT_CONFIG_PATH)


    handler.start(config)