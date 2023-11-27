from backend_logging import get_logger

class VisionClient():
    """
    This class is responsible for connecting to the vision camera,
    requesting images and performing image transformation and calculation
    """
    def __init__(self):
        pass

    def do_vision_thing(self):
        """
        - take image (from home position)
        - detect crates on image
         - display image
         - if no crates -> retake
           - if no crates -> error
         - if assistance crate on image -> send status assitance and end command
        - if crates:
         - identify highest crate
         - calculate coordinates + method (suction/hook)
         (maybe return all coordinates + methods)
        - return errors + coordinates + methods
        """
        pass