import urx
from vision.vision_client import VisionClient

class RobotController():
    """
    This class is responsible for connecting to the 
    """
    def __init__(self):
        self.ip = None
        self.vision_client = None
    
    def connect(self, ip):
        self.rob = urx.Robot(ip)
        self.vision_client = VisionClient()