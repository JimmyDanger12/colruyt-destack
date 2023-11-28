import urx
from vision.vision_client import VisionClient
from robot_mechanics.status import Status
from backend_logging import get_logger
import logging
import time

class RobotController():
    """
    This class is responsible for connecting the main script to the robot and
    executing robot commands
    """
    def __init__(self, ip, handler):
        self.handler = handler
        self.ip = ip
        self.vision_client = VisionClient() #maybe move to connect phase
        self.rob = None
    
    def connect(self):
        try:
            self.rob = urx.Robot(self.ip)
            status = Status.Connected
            get_logger(__name__).log(logging.INFO,
                            "New client connected")
        except Exception as e:
            get_logger(__name__).log(logging.WARNING,
                                     f"Error when connecting to robot: {e}")
            status = Status.Disconnected       
        self._change_status(status)   
    
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
        try:
            l = 0.02
            v = 0.05
            a = 0.3
            self.rob.set_tcp((0,0,0,0,0,0))
            pose = self.rob.getl()
            pose[2] += l
            self.rob.movel(pose,acc=a,vel=v)
        except Exception as e:
            get_logger(__name__).log(logging.ERROR,
                                     f"Error when executing robot command {e}")  
            self.rob.close()   

    def stop(self,priority):
        """
        Hard stop robot
        -> Status Stopped -> priority 1 = high, 2 = low
        """
        self.rob.close()
        self._change_status(Status.Stopped)

    def destack_done(self):
        """
        Soft stop robot
        -> Status Done
        """
        self._change_status(Status.Done)

    def _change_status(self, status):
        self.handler.change_status(status)