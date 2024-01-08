import urx
from vision.vision_client import VisionClient
from robot_mechanics.status import Status
from backend_logging import get_logger
import logging
import time

DIG_OUT_CYL_BOT = 1
DIG_OUT_CYL_SLI = 2
DIG_IN_DROPOFF = 3
ANA_IN_PRESSR = 4
ANA_IN_PRESSL = 5
ANA_OUT_CONV = 6

class RobotController():
    """
    This class is responsible for connecting the main script to the robot and
    executing robot commands
    """
    def __init__(self, ip, tcp, handler):
        self.handler = handler
        self.ip = ip
        self.vision_client = VisionClient() #maybe move to connect method
        self.rob = None
        self.tcp = tcp
        self.robot_position = None #use? or always getl in movement functions
        self.pick_loc = None

        self.tcp_hooks = [0, 0.05583, 0.0915, 0, 0, 0]
        self.tcp_bar = [0, 0.02, 0.1095, 0, 0, 0]
        self.tcp_plate = [0, -0.125, 0.0895, 0, 0, 0]

        self.pre_pick = [0.055, -0.708, 0.331, 1.2, -1.2, 1.2] 
        self.place_pos = [0.72478, 0.39171, 0.0236, -0.006, -1.59, 0.02] 
        self.pre_place = [-0.72478, 0.39171, 0.2236, -0.006, -1.59, 0.02]
        self.via_pose = [-0.612, -0.305, 0.340, 0.55, -1.5, 0.56]
        #TODO: set robot home position
    
    def connect(self):
        try:
            self.rob = urx.Robot(self.ip)
            self.rob.set_tcp(self.tcp)

            status = Status.Connected
            get_logger(__name__).log(logging.INFO,
                            "Robot connected")
            #self.vision_client.connect()
        except Exception as e:
            get_logger(__name__).log(logging.WARNING,
                 f"Error when connecting to robot: {e}")
            status = Status.Disconnected       
        self._change_status(status)   
    
    def start_destack(self):

        while True:
            
            self.move_start_pos(self.tcp_bar) 
            pick_coords, pick_loc, pick_ori, picked_ori, crate_size = self.retrieve_pick_pos()
            if pick_coords == []:
                break #alert and log
            self.move_pre_pick_pos() 
            self.move_pre_picked_pos(pick_loc, pick_ori) 
            self.move_picked_pos(pick_loc, pick_ori, picked_ori) 
            self.move_out_carrier(pick_loc, picked_ori) 
            self.depl_safety_syst()
            self.move_pre_place_pos(picked_ori) 
            self.retr_safety_syst()
            self.move_on_conv_pos(crate_size, picked_ori) 
            self.move_place_crate() 
            dropoff_value = self.rob.get_digital_in(DIG_IN_DROPOFF)
            if dropoff_value == True:
                self.turn_conv_on()
            else: 
                break #add alert
        return
    

    def move_start_pos(self, tcp_bar):
        get_logger(__name__).log(logging.DEBUG,
            f"Starting move to start pos")
        self.rob.set_tcp(tcp_bar)
        get_logger(__name__).log(logging.DEBUG,
            f"set tcp")
        print(self.pre_place, type(self.pre_place))
        self.rob.movel(self.pre_place, acc=1, vel=0.5)
        get_logger(__name__).log(logging.DEBUG,
            f"Move start pos completed")

    def retrieve_pick_pos(self):
        get_logger(__name__).log(logging.DEBUG,
            f"Retrieval of pick position started")
        pick_coords = [-0.17534, -1.10184, -0.011093, -8.8043, 1.0891, -5.7366] #TODO: get coords from vision
        crate_size = 0.33
        pick_loc = pick_coords[0:2]
        pick_ori = pick_coords[3:5]
        picked_ori = [pick_ori[0], pick_ori[1], pick_ori[2] -20]
        get_logger(__name__).log(logging.DEBUG,
            f"Retreived coords, crate_size from vision {pick_coords}, {crate_size}")
        pick_coords[3] += -20 #TODO: why here?
        return pick_coords, crate_size, pick_loc, pick_ori, picked_ori

    def move_pre_pick_pos(self):
        get_logger(__name__).log(logging.DEBUG,
            f"starting move to pre pick pos")
        self.rob.movec(self.via_pose, self.pre_pick, acc=1, vel=0.5, r=0.2, mode=1)
        get_logger(__name__).log(logging.DEBUG,
            f"completed move to pre pick pos")
    
    def move_pre_picked_pos(self, pick_loc, pick_ori):
        get_logger(__name__).log(logging.DEBUG,
            f"starting move to pre picked")
        self.rob.movel(self.pre_pick[0:2] + pick_ori, acc=1, vel= 0.25)
        self.rob.movel([pick_loc[0], pick_loc[1], pick_loc[2]+0.03, pick_ori[0]+10, pick_ori[1], pick_ori[2]], acc=1, vel= 0.25)
        get_logger(__name__).log(logging.DEBUG,
            f"completed move to pre picked")

    def move_picked_pos(self, pick_loc, pick_ori):
        get_logger(__name__).log(logging.DEBUG,
            f"starting picking")
        self.rob.movel(pick_loc + [pick_ori[0]+10, pick_ori[1], pick_ori[2]], acc=1, vel= 0.1)
        self.rob.movel(pick_loc + pick_ori, acc=1, vel= 0.1)
        self.rob.set_tcp(self.tcp_plate)
        get_logger(__name__).log(logging.DEBUG,
            f"set tcp to {self.tcp_plate}")
        
        tilt_movement = [0,0,0,-20,0,0]
        get_logger(__name__).log(logging.DEBUG,
            f"Attempting to move (rel) by {tilt_movement}")
        self.rob.movel(tilt_movement, #Rotate around plate
                       acc=0.5, 
                       vel= 0.1, 
                       relative=True)

        get_logger(__name__).log(logging.DEBUG,
            "Move picked_pos completed")

    def move_out_carrier(self, pick_loc, picked_ori):
        get_logger(__name__).log(logging.DEBUG,
            f"staring move out carrier")
        self.rob.set_tcp(self.tcp_bar)
        self.rob.movel([0, 0, 0.03, 0, 0, 0], acc=0.5, vel= 0.1, wait=False, relative=True)
        self.rob.movel([pick_loc[0], self.pre_pick[1], pick_loc[2]+0.03] + picked_ori, acc=1, vel= 0.25, wait=False)
        while True:
            pressure_value = self.rob.get_analog_in(ANA_IN_PRESSR) + self.rob.get_analog_in(ANA_IN_PRESSL)
            if pressure_value < 800:
                self.stop()
            if self.rob.get_pose() == [pick_loc[0], self.pre_pick[1], pick_loc[2]+0.03] + picked_ori:
                break
        get_logger(__name__).log(logging.DEBUG,
            f"completed move out carrier")


    def depl_safety_syst(self):
        get_logger(__name__).log(logging.DEBUG,
            f"deploying safety system")
        self.rob.set_digital_out(DIG_OUT_CYL_BOT, True)
        time.sleep(3)
        self.rob.set_digital_out(DIG_OUT_CYL_SLI, True)
        time.sleep(3)
        self.rob.set_digital_out(DIG_OUT_CYL_BOT, False)
        get_logger(__name__).log(logging.DEBUG,
            f"deployed safety system")

    def move_pre_place_pos(self, picked_ori):
        get_logger(__name__).log(logging.DEBUG,
            f"starting move to pre pick")
        self.rob.movel(self.pre_pick[0:2] + picked_ori, acc=1, vel= 1)
        self.rob.movec(self.via_pose[0:2] + picked_ori, self.pre_place[0:2] + picked_ori, acc=1, vel=0.5, r=0, mode=1)
        get_logger(__name__).log(logging.DEBUG,
            f"completed move to pre pick")

    def retr_safety_syst(self):
        get_logger(__name__).log(logging.DEBUG,
            f"retracting safety system")
        self.rob.set_analog_out(ANA_OUT_CONV, 0)
        self.rob.set_digital_out(DIG_OUT_CYL_BOT, True)
        time.sleep(3)
        self.rob.set_digital_out(DIG_OUT_CYL_SLI, False)
        time.sleep(3)
        self.rob.set_digital_out(DIG_OUT_CYL_BOT, False)
        get_logger(__name__).log(logging.DEBUG,
            f"retracted safety system")

    def move_on_conv_pos(self, crate_size, picked_ori):
        get_logger(__name__).log(logging.DEBUG,
            f"starting move on conveyer")
        self.rob.movel([self.place_pos[0], self.place_pos[1], self.place_pos[2]+crate_size-20] + picked_ori, acc=1, vel= 1)
        get_logger(__name__).log(logging.DEBUG,
            f"completed move on conveyer")

    def move_place_crate(self):
        get_logger(__name__).log(logging.DEBUG,
            f"starting move place crate")
        self.rob.movel(self.place_pos, acc=1, vel= 1)
        time.sleep(5)
        self.rob.movel(self.pre_place, acc=1, vel= 1)
        get_logger(__name__).log(logging.DEBUG,
            f"completed move place crate")

    def turn_conv_on(self):
        get_logger(__name__).log(logging.DEBUG,
            f"drop off comfirmed, turning on conveyer")
        self.rob.set_analog_out(ANA_OUT_CONV, 1)
        get_logger(__name__).log(logging.DEBUG,
            f"turned conveyer on")



    def stop(self,priority):
        """
        Hard stop robot
        -> Status Stopped -> priority 1 = high, 2 = low
        """
        #self.rob.close()
        self._change_status(Status.Stopped)

    def destack_done(self):
        """
        Soft stop robot
        -> Status Done
        """
        self._change_status(Status.Done)

    def _change_status(self, status):
        self.handler.change_status(status)