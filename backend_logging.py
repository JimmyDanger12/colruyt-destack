from logging.handlers import RotatingFileHandler
from logging import Logger

import os
import logging
import sys

# default values for file logging
DEFAULT_LOG_DIR = "logs"
DEFAULT_LOG_FILE_NAME = "backend.log"
DEFAULT_LOG_LEVEL = logging.WARNING

# format of log messages
LOG_FORMAT = '[%(asctime)s] - [%(name)s] - [%(levelname)s] - %(message)s'

# configs of file logging
MAX_BYTES = 10*1024*1024
BACKUP_COUNT = 5

# global variables
config = None
loggers = {}
file_log_handler = None
console_log_handler = None

def setup_logging(t_config):
    global config
    config = t_config

def attach_log_handler(logger: Logger):
    # if no config is set exit - then nothing is logged for tests
    if config is None:
        return
    # create main log handler
    global file_log_handler
    global console_log_handler

    # Log messages in file
    if file_log_handler is None:
        log_dir = DEFAULT_LOG_DIR
        log_file_name = DEFAULT_LOG_FILE_NAME
        log_level = DEFAULT_LOG_LEVEL
        if config is not None:
            log_dir = config['LOGGING', 'log_dir', log_dir]
            log_file_name = config['LOGGING', 'log_name', log_file_name]
            log_level = int(config['LOGGING', 'min_level', log_level])
        file_log_handler = RotatingFileHandler(
            os.path.join(log_dir, log_file_name), 
            maxBytes=MAX_BYTES, 
            backupCount=BACKUP_COUNT
        )
        logFormatter = logging.Formatter(LOG_FORMAT)
        file_log_handler.setFormatter(logFormatter)
        file_log_handler.setLevel(log_level)

    # Log messages on console
    if console_log_handler is None:
        console_log_handler = logging.StreamHandler(sys.stdout)
        logFormatter = logging.Formatter(LOG_FORMAT)
        console_log_handler.setFormatter(logFormatter)
        console_log_level = config['LOGGING','console_log_level']
        # set log level
        console_log_handler.setLevel(int(console_log_level))

    # attach logger
    if logger is not None:
        logger.addHandler(file_log_handler)
        env_debug = eval(config['LOGGING','debug'])
        if env_debug:
            logger.addHandler(console_log_handler)

def get_logger(logger_name: str = 'default'):
    loggername = 'rocsys.{}'.format(logger_name)

    if loggers.get(loggername):
        return loggers.get(loggername)

    logger = logging.getLogger(loggername)
    logger.setLevel(1) # log levels of handlers are higher anyway
    attach_log_handler(logger)
    loggers[loggername] = logger
    return logger
