from configparser import RawConfigParser
from pathlib import Path
import os

class GlobalConfig:
    fileFound = False
    configFilePath = None
    configContent = None

    def __init__(self, config_file):
        self.fileFound = config_file
        # check if file exists, if not mark as not found
        path = Path(config_file)
        if path.is_file():
            self.configContent = RawConfigParser()
            self.configContent.read(config_file)
            self.fileFound = True

    def __getitem__(self, pos):
        if type(pos) != tuple or len(pos) < 2 or len(pos) > 3:
            raise IndexError('Index must be a tuple of length 2 (section, property) or 3 (section, property, defaultValue')
        if len(pos) == 2:
            section, property = pos
            defaultValue = None
            defaultValueDefined = False
        if len(pos) == 3:
            section, property, defaultValue = pos
            defaultValueDefined = True
        
        # get env var if  avaialble and return
        envVal = os.getenv('{}__{}'.format(section, property))
        if envVal != None:
            return envVal

        # if not found, and file exists, try get from file
        if self.fileFound and self.configContent.has_option(section, property):
            return self.configContent[section][property]

        # value not found up to now => check if default was defined
        if defaultValueDefined:
            return defaultValue

        raise IndexError('section {} -> parameter {} not found'.format(section, property))