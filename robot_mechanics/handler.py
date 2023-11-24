from robot_mechanics.robot_controller import RobotController
from globalconfig import GlobalConfig
from user_interface.user_interface import UI
from backend_logging import setup_logging, get_logger
import logging
import time

FIELD_ROBOT = "ROBOT"
FIELD_IP = "ip"

class Handler():
    """
    This is the main class responsible for connecting to all necessary resources
    and distributing messages correctly.
    """
    def __init__(self):
        self.robot_controller = None
        self.ui = None

    def start(self, config:GlobalConfig):
        setup_logging(config)
        get_logger(__name__).log(100,
                                 f"Robot Handler starting...")
        
        robot_ip = config[FIELD_ROBOT,FIELD_IP]
        self.robot_controller = RobotController()
        self.ui = UI()

        self.ui.start_ui()
        print("Monkey")
        time.sleep(10)
        get_logger(__name__).log(logging.WARNING,f"New log message")
        #self.robot_controller.connect(robot_ip)