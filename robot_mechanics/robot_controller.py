import urx
from vision.vision_client import VisionClient
from robot_mechanics.status import Status
from backend_logging import get_logger
import logging
import time

class RobotController():
    """
    This class is responsible for connecting to the 
    """
    def __init__(self, ip, handler):
        self.handler = handler
        self.ip = ip
        self.vision_client = None
        self.rob = None
    
    def connect(self):
        try:
            self.rob = urx.Robot(self.ip)
            status = Status.Connected
        except Exception as e:
            get_logger(__name__).log(logging.WARNING,
                                     f"Error when connecting to robot: {e}")
            status = Status.Disconnected       
        self._change_status(status)   
        self.vision_client = VisionClient()
    
    def start_destack(self):
        """
        Command structure:
        - change status to running
        - take image -> Vision client -> returns coords, method (suction/hook), error (error or assitance req)
        - move to coords
        - start suction/hook
        - lift up
        - move backwards
        - move to conveyor
        (- retake image?)
        """
        self._change_status(Status.Running)
        time.sleep(3)
        self._change_status(Status.Done)

    def stop(self):
        """
        Hard stop robot
        -> Status Stopped
        """
        self._change_status(Status.Stopped)

    def destack_done(self):
        """
        Soft stop robot
        -> Status Done
        """
        self._change_status(Status.Done)

    def _change_status(self, status):
        self.handler.change_status(status)