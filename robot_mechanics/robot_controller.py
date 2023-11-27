import urx
from vision.vision_client import VisionClient
from robot_mechanics.status import Status

class RobotController():
    """
    This class is responsible for connecting to the 
    """
    def __init__(self, ip, handler):
        self.handler = handler
        self.ip = ip
        self.vision_client = None
    
    def connect(self):
        #self.rob = urx.Robot(self.ip)
        #self.vision_client = VisionClient()
        self._change_status(Status.Connected)
        pass
    
    def start_destack(self):
        pass

    def stop(self):
        pass

    def destack_done(self):
        pass

    def _change_status(self, status):
        self.handler.change_status(status)