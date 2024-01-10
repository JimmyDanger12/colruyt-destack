import urx
from vision.vision_client import NoPickUpCrateException, NoDetectedCratesException, VisionClient
from robot_mechanics.status import Status
from backend_logging import get_logger
import logging
import time
import numpy

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
    def __init__(self, ip, handler):
        self.handler = handler
        self.ip = ip
        self.vision_client = VisionClient()
        self.rob = None
        

        self.tcp_bar = [0.02, 0, 0.1095, 0, 0, 0]

        self.pre_pick = [-0.055, -0.708, 0.331, 1.2, -1.2, 1.2] 
        self.post_pick = [-0.12111, -0.40713, 0.37773, 0.8309, -0.8033, 1.4556]
        self.place_pos = [-0.70156, 0.40176, 0.01387, -0.0063, -1.5466, 0.0049] 
        self.start_pos = [-1.0433, 0.40644, 0.4366, -0.0462, -1.4907, 0.00434]
        self.pre_place = [-0.67659, 0.39188, 0.21248, 0.0134, -1.1911, 0.0044]
        self.via_pose = [-0.612, -0.305, 0.340, 0.55, -1.5, 0.56]
        self.post_via_pose = [-0.57545, -0.27388, 0.32781, 0.4016, -1.1016, 0.6319]
    
    def connect(self):
        try:
            self.rob = urx.Robot(self.ip)
            status = Status.Connected
            get_logger(__name__).log(logging.INFO,
                "Robot connected")
            self.vision_client.connect()
        except Exception as e:
            get_logger(__name__).log(logging.WARNING,
                f"Error when connecting to robot: {e}")
            status = Status.Disconnected       
        self._change_status(status)   

    def test_vision(self):
        self.rob.set_tcp(self.tcp_bar)
        self.move_start_pos()
        pick_loc, pick_ori, crate_height = self.retrieve_pick_pos()
        self.move_pre_pick_pos()
        self.move_pre_picked_pos(pick_loc, pick_ori)
        self.move_picked_pos()
    
    def test_drop_off(self):
        self.rob.set_tcp(self.tcp_bar)
        self.rob.movel(self.pre_place,
                       acc=1,
                       vel=0.05)
        crate_height = 0.196
        #pick_loc, pick_ori, crate_height = self.retrieve_pick_pos()
        self.move_on_conv_pos(crate_height)
        self.move_place_crate()
    
    def start_destack(self):
        alerted = False
        while True:
            self.rob.set_tcp(self.tcp_bar)
            self.move_start_pos()
            try:
                pick_loc, pick_ori, crate_height = self.retrieve_pick_pos()
            except NoDetectedCratesException:
                get_logger(__name__).log(logging.INFO,
                    "No Crates detected")
                break
            except NoPickUpCrateException:
                get_logger(__name__).log(logging.INFO,
                    "Unpickable crate detected")
                alerted = True
                self.alert_worker()
                break
            self.move_pre_pick_pos() 
            self.move_pre_picked_pos(pick_loc, pick_ori) 
            self.move_picked_pos() 
            self.move_out_carrier(pick_loc) 
            #self.depl_safety_syst()
            self.move_pre_place_pos() 
            #self.retr_safety_syst()
            self.move_on_conv_pos(crate_height) 
            self.move_place_crate() 
            """dropoff_value = self.rob.get_digital_in(DIG_IN_DROPOFF)
            if dropoff_value == True:
                self.turn_conv_on()
            else: 
                break #add alert"""
            get_logger(__name__).log(logging.INFO,
                                     "Crate placed successfully")
        if not alerted:
            get_logger(__name__).log(logging.INFO,
                                    "Done/No Boxes detected")
            self._change_status(Status.Done)
    
    def alert_worker(self):
        self._change_status(Status.Alerted)

    def move_start_pos(self):
        """
        This function contains:
        - move to start position 
        - set the tool center point 
        """
        get_logger(__name__).log(logging.DEBUG,
            f"Starting move to start pos")
        self.rob.set_tcp(self.tcp_bar) 
        get_logger(__name__).log(logging.DEBUG,
            f"set tcp")
        self.rob.movel(self.start_pos, acc=1, vel=0.1) 
        get_logger(__name__).log(logging.DEBUG,
            f"Move start pos completed")
        

    def retrieve_pick_pos(self):
        """
        This function contains:
        -  
        """
        get_logger(__name__).log(logging.DEBUG,
            f"Retrieval of pick position started")
        pick_coords, crate_height = self.vision_client.get_valid_pickup_loc()
        """pick_coords = [-0.18016, -1.05289, -0.11338, 1.24, -1.2, 1.24] 
        crate_height = 0.33"""
        pick_coords = [round(c,5) for c in pick_coords]
        get_logger(__name__).log(logging.INFO,
            f"Retrieved coords, crate_size from vision {pick_coords}, {crate_height}")
        pick_loc = pick_coords[0:3]
        pick_ori = [1.24, -1.2, 1.24]
        get_logger(__name__).log(logging.DEBUG,
            f"Retrieval of pick coordinates completed")
        return pick_loc, pick_ori, crate_height
    

    def move_pre_pick_pos(self):
        """
        This function contains the movements:
        - move to pre-pick position
        """
        get_logger(__name__).log(logging.DEBUG,
            f"starting move to pre pick pos")
        self.rob.movels([self.via_pose, self.pre_pick], acc=1, vel=0.1, radius=0.05)
        get_logger(__name__).log(logging.DEBUG,
            f"completed move to pre pick pos")
        
    
    def move_pre_picked_pos(self, pick_loc, pick_ori):
        """
        This function contains the movements:
        - move to just above pick position
        - tilt forward to allow movement on to crate
        """
        get_logger(__name__).log(logging.DEBUG,
            f"starting move to pre picked")
        self.rob.movel([pick_loc[0], pick_loc[1], pick_loc[2]+0.03] + pick_ori, acc=1, vel=0.025)
        self.rob.movel_tool([0, 0, 0, 0, -0.35, 0], acc=1, vel=0.01)
        get_logger(__name__).log(logging.DEBUG,
            f"completed move to pre picked")
        

    def move_picked_pos(self):
        """
        This function contains the movements:
        - move down on on the crate
        - tilt back around the hooks to vertical position
        - tilt back around the bottom of the plate to pick up crate
        """
        get_logger(__name__).log(logging.DEBUG,
            f"starting picking")
        self.rob.movel([0, 0, -0.0475, 0, 0, 0], acc=1, vel=0.01, relative=True)
        movement = [-0.005, 0 , 0.005, 0, 0.35, 0] #move around hooks
        self.rob.movel_tool(movement, acc=1, vel=0.01)
        movement = [-0.01, 0, -0.05, 0, 0.35, 0] #move around plate
        movement = [x*0.8 for x in movement]
        self.rob.movel_tool(movement)
        get_logger(__name__).log(logging.DEBUG,
            "Move picked_pos completed")
        

    def move_out_carrier(self, pick_loc):
        """
        This function contains the movements:
        - move the crate up 
        - move the crate back out of the carrier
        """
        get_logger(__name__).log(logging.DEBUG,
            f"staring move out carrier")
        self.rob.movel([0, 0, 0.03, 0, 0, 0], acc=0.5, vel=0.01, relative=True)
        if pick_loc[3] >= 0.01 :
            self.rob.movel([pick_loc[1], -400, pick_loc[3]] + self.post_pick[3:6], acc=1, vel=0.025)
        elif pick_loc [3] < 0.01 : 
            self.rob.movel([pick_loc[1], -400, self.post_pick] + self.post_pick[3:6], acc=1, vel=0.025)
        #self.rob.movel([pick_loc[1], self.post_pick[2], pick_loc[3]] + self.post_pick[3:6], acc=1, vel=0.025)
        #self.rob.movel([0, 0.40, 0, 0, 0, 0], acc=0.5, vel=0.025, relative=True)
        """while True: #TODO: add pressure sensor
            pressure_value = self.rob.get_analog_in(ANA_IN_PRESSR) + self.rob.get_analog_in(ANA_IN_PRESSL)
            if pressure_value < 800:
                self.stop()
            if self.rob.get_pose() == [] :
                break"""
        get_logger(__name__).log(logging.DEBUG,
            f"completed move out carrier")


    def depl_safety_syst(self):
        """
        This function contains:
        - expaning the bottom plate
        - expanding the safety sliders
        - retracting the bottom plate
        """
        get_logger(__name__).log(logging.DEBUG,
            f"deploying safety system")
        self.rob.set_digital_out(DIG_OUT_CYL_BOT, True)
        time.sleep(3)
        self.rob.set_digital_out(DIG_OUT_CYL_SLI, True)
        time.sleep(3)
        self.rob.set_digital_out(DIG_OUT_CYL_BOT, False)
        get_logger(__name__).log(logging.DEBUG,
            f"deployed safety system")
        

    def move_pre_place_pos(self):
        """
        This function contains the movements:
        - move to post pick 
        - move to pre place 
        """
        get_logger(__name__).log(logging.DEBUG,
            f"starting move to pre place")
        print("post_pick")
        self.rob.movel(self.post_pick, acc=1, vel=0.025)
        print("pre_place")
        self.rob.movels([self.post_via_pose,self.pre_place], acc=1, vel=0.05, radius=0.05)
        get_logger(__name__).log(logging.DEBUG,
            f"completed move to pre place")
        

    def retr_safety_syst(self):
        """
        This function contains:
        - turning the conveyer off
        - expaning bottom plate 
        - retracting safety sliders
        - retracting bottom plate
        """
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
        

    def move_on_conv_pos(self, crate_height):
        """
        This function contains the movements:
        - move crate on conveyer
        """
        get_logger(__name__).log(logging.DEBUG,
            f"starting move on conveyer")
        pos = self.place_pos
        movement = crate_height - 0.185
        print("Drop_down movement",movement)
        pos[2] += movement
        self.rob.movel(pos, acc=1, vel=0.025)
        get_logger(__name__).log(logging.DEBUG,
            f"completed move on conveyer")
        

    def move_place_crate(self):
        """
        This function contains the movements:
        - tilt foward around the hooks to place the crate
        - move up from the placed crate 
        """
        get_logger(__name__).log(logging.DEBUG,
            f"starting move place crate")
        movement = [0.004, 0 , -0.004, 0, -0.25, 0] #move around hooks
        self.rob.movel_tool(movement, acc=1, vel=0.01)
        self.rob.movel([0,0,0.2,0,0,0], acc=1, vel=0.025, relative=True)
        get_logger(__name__).log(logging.DEBUG,
            f"completed move place crate")
        

    def turn_conv_on(self):
        """
        This function contains:
        - turing the conveyer on 
        """
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
        self._change_status(Status.Stopped)
        #TODO: Add function to stop robot

    def destack_done(self):
        """
        Soft stop robot
        -> Status Done
        """
        self._change_status(Status.Done)
        #TODO: remove / add functionality

    def _change_status(self, status):
        self.handler.change_status(status)