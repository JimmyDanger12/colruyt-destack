import urx
from vision.vision_client import NoPickUpCrateException, NoDetectedCratesException, VisionClient
from robot_mechanics.status import Status
from backend_logging import get_logger
import logging
import time
import numpy
import copy
import pyfirmata2 as pyfirmata

DIG_OUT_CONV = 4
DIG_IN_DROPOFF = 3
DIG_OUT_CYL_BOT = 3
ANA_IN_PRESSR = None
DIG_OUT_ACT_SLI_EXT = None
DIG_OUT_ACT_SLI_RETR = None

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
        self.place_pos = [-0.70156, 0.38036, 0.01387, 0.0, -1.6, 0.0] 
        self.start_pos = [-1.0433, 0.40644, 0.4366, 0.0, -1.6, 0.0]
        self.pre_place = [-0.67659, 0.39188, 0.21248, 0.0134, -1.1911, 0.0044]
        self.via_pose = [-0.612, -0.305, 0.340, 0.55, -1.5, 0.56]
        self.post_via_pose = [-0.57545, -0.27388, 0.32781, 0.4016, -1.1016, 0.6319]
    
    def connect(self):
        try:
            self.rob = urx.Robot(self.ip)
            status = Status.Connected
            self.vision_client.connect()
            self.connect_arduino()
            get_logger(__name__).log(logging.INFO,
                "Robot connected")
        except Exception as e:
            get_logger(__name__).log(logging.WARNING,
                f"Error when connecting to robot: {e}")
            status = Status.Disconnected       
        self._change_status(status)   

    def connect_arduino(self):
        board = pyfirmata.Arduino('COM3')
        it = pyfirmata.util.Iterator(board)
        it.start()

        global DIG_OUT_ACT_SLI_EXT
        DIG_OUT_ACT_SLI_EXT = board.digital[7]
        global DIG_OUT_ACT_SLI_RETR
        DIG_OUT_ACT_SLI_RETR = board.digital[8]
        DIG_OUT_ACT_SLI_EXT.write(0) 
        DIG_OUT_ACT_SLI_RETR.write(0)
        global ANA_IN_PRESSR
        ANA_IN_PRESSR = board.get_pin('a:0:i')
        global ANA_IN_PRESSL
        ANA_IN_PRESSL = board.get_pin('a:1:i')
    
    def test_vision(self):
        self.rob.set_tcp(self.tcp_bar)
        self.move_start_pos()
        pick_loc, pick_ori, crate_height = self.retrieve_pick_pos()
        self.move_pre_pick_pos()
        self.move_pre_picked_pos(pick_loc, pick_ori)
        #self.move_picked_pos()
    
    def test_drop_off(self):
        self.move_start_pos()
        self.depl_safety_syst()
        time.sleep(5)
        self.retr_safety_syst()
    
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
                self.alert_worker("NoPickupCrate","Heighest Crate is a non-pickable Crate")
                break
            self.move_pre_pick_pos() 
            self.move_pre_picked_pos(pick_loc, pick_ori) 
            self.move_picked_pos() 
            try:
                self.move_out_carrier(pick_loc) 
            except Exception as e:
                print("Exception",e)
                break
            #self.depl_safety_syst()
            self.move_pre_place_pos()
            #self.retr_safety_syst()
            self.move_on_conv_pos(crate_height) 
            self.move_place_crate() 
            dropoff_value = self.rob.get_digital_in(DIG_IN_DROPOFF)
            if dropoff_value == True:
                self.turn_conv_on()
            else: 
                get_logger(__name__).log(logging.INFO,
                    f"drop off not confirmed, alerting worker")
                alerted = True
                self.alert_worker("NoDropOff", "Crate was not dropped off")
                break
            get_logger(__name__).log(logging.INFO,
                "Crate placed successfully")
        if not alerted:
            get_logger(__name__).log(logging.INFO,
                "Done/No Boxes detected")
            self._change_status(Status.Done)
    
    def alert_worker(self,message_type=None,message=None):
        data = {'message_type': message_type, 'message':message}
        self._change_status(Status.Alerted, data)

    def move_start_pos(self):
        """
        This function contains:
        - move to start position 
        - set the tool center point 
        """
        get_logger(__name__).log(logging.INFO,
            f"Starting move to start pos")
        self.rob.set_tcp(self.tcp_bar) 
        get_logger(__name__).log(logging.DEBUG,
            f"set tcp")
        self.rob.movel(self.start_pos, acc=1, vel=0.25) 
        get_logger(__name__).log(logging.INFO,
            f"Completed move to start pos")
        

    def retrieve_pick_pos(self):
        """
        This function contains:
        -  retrieving the coordinates and orientation of the to be picked crate
        """
        get_logger(__name__).log(logging.DEBUG,
            f"Retrieval of pick position started")
        pick_coords, crate_height = self.vision_client.get_valid_pickup_loc()
        pick_coords = [round(c,5) for c in pick_coords]
        get_logger(__name__).log(logging.INFO,
            f"Retrieved coords, crate_size from vision {pick_coords}, {crate_height}")
        pick_loc = pick_coords[0:3]
        #pick_ori = pick_coords[3:6]
        pick_ori = [1.209, -1.209, 1.209]
        get_logger(__name__).log(logging.DEBUG,
            f"Retrieval of pick coordinates completed")
        return pick_loc, pick_ori, crate_height
    

    def move_pre_pick_pos(self):
        """
        This function contains the movements:
        - move to pre-pick position
        """
        get_logger(__name__).log(logging.INFO,
            f"Starting move to pre pick pos")
        self.rob.movels([self.via_pose, self.pre_pick], acc=1, vel=0.4, radius=0.05)
        get_logger(__name__).log(logging.INFO,
            f"Completed move to pre pick pos")
        
    
    def move_pre_picked_pos(self, pick_loc, pick_ori):
        """
        This function contains the movements:
        - move to just above pick position
        - tilt forward to allow movement on to crate
        """
        get_logger(__name__).log(logging.INFO,
            f"Starting move to pre picked")
        self.rob.movel([pick_loc[0], pick_loc[1], pick_loc[2]+0.03] + pick_ori, acc=1, vel=0.2)
        self.rob.movel_tool([0, 0, 0, 0, -0.35, 0], acc=1, vel=0.1)
        get_logger(__name__).log(logging.INFO,
            f"Completed move to pre picked")
        

    def move_picked_pos(self):
        """
        This function contains the movements:
        - move down on on the crate
        - tilt back around the hooks to vertical position
        - tilt back around the bottom of the plate to pick up crate
        """
        get_logger(__name__).log(logging.INFO,
            f"Starting crate pick")
        self.rob.movel([0, 0, -0.035, 0, 0, 0], acc=1, vel=0.05, relative=True)
        movement = [-0.005, 0 , 0.005, 0, 0.35, 0] #move around hooks
        self.rob.movel_tool(movement, acc=1, vel=0.05)
        movement = [-0.01, 0, -0.05, 0, 0.35, 0] #move around plate
        movement = [x*0.8 for x in movement]
        self.rob.movel_tool(movement, acc=1, vel=0.02)
        get_logger(__name__).log(logging.INFO,
            "Completed crate pick")
        

    def move_out_carrier(self, pick_loc):
        """
        This function contains:
        - move the crate up 
        - checking with the pressure sensors if the crate has been picked and not fallen down 
        - move the crate back out of the carrier
        """
        get_logger(__name__).log(logging.INFO,
            f"Starting move out carrier")
        self.rob.movel([0, 0, 0.015, 0, 0, 0], acc=0.5, vel=0.01, relative=True)
        goal_pos = []
        if pick_loc[2] >= 0.10 :
            get_logger(__name__).log(logging.DEBUG,
            "crate above base")
            goal_pos = [pick_loc[0], -0.55, pick_loc[2]] + self.post_pick[3:6]
            self.rob.movel(goal_pos, acc=1, vel=0.1, wait=False)
        elif pick_loc [2] < 0.10 : 
            get_logger(__name__).log(logging.DEBUG,
            "crate lower than base")
            goal_pos = [pick_loc[0], -0.55, self.post_pick[2]] + self.post_pick[3:6]
            self.rob.movel(goal_pos, acc=1, vel=0.1, wait=False)

        def are_coords_within_tolerance(current_pos,goal_pos, tolerance):
            return all(
                abs(coord - goal_coord) <= tolerance for coord, goal_coord in zip(current_pos,goal_pos))
        
        get_logger(__name__).log(logging.DEBUG,
            f"Starting reading pressure")
        alerted=False
        while True:
            pressure_value = 0.8 #ANA_IN_PRESSR.read() # +ANA_IN_PRESSL.read()
            if pressure_value < 0.600:
                get_logger(__name__).log(logging.WARNING,
                    f"Pressure loss, alerting worker")
                alerted=True
                self.stop()
                self.alert_worker("NoPressure", "Pressure lost - Crate not gripped")
                break
            current_pos = self.rob.getl(wait=True)
            if are_coords_within_tolerance(current_pos[:3],goal_pos[:3], 0.01):
                break
        get_logger(__name__).log(logging.DEBUG,
            f"Stopped reading pressure")
        if alerted:
            raise Exception("Pressure lost")
        else:
            get_logger(__name__).log(logging.INFO,
                f"Completed move out carrier")


    def depl_safety_syst(self):
        """
        This function contains:
        - expaning the bottom plate
        - expanding the safety sliders
        - retracting the bottom plate
        """
        get_logger(__name__).log(logging.DEBUG,
            f"Started deploying safety system")
        self.rob.set_digital_out(DIG_OUT_CYL_BOT, 0)
        time.sleep(1)
        DIG_OUT_ACT_SLI_EXT.write(1) 
        time.sleep(3.5) 
        DIG_OUT_ACT_SLI_EXT.write(0)
        self.rob.set_digital_out(DIG_OUT_CYL_BOT, 1)
        time.sleep(3) 
        get_logger(__name__).log(logging.DEBUG,
            f"Completed deploying safety system")
        

    def move_pre_place_pos(self):
        """
        This function contains the movements:
        - move to post pick 
        - move to pre place 
        """
        get_logger(__name__).log(logging.INFO,
            f"Starting move to pre place")
        self.rob.movel(self.post_pick, acc=1, vel=0.1)
        self.rob.movels([self.post_via_pose,self.pre_place], acc=1, vel=0.2, radius=0.1)
        get_logger(__name__).log(logging.INFO,
            f"Completed move to pre place")
        

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
        self.rob.set_digital_out(DIG_OUT_CYL_BOT, 0)
        time.sleep(1)
        DIG_OUT_ACT_SLI_RETR.write(1) 
        time.sleep(3.5) 
        DIG_OUT_ACT_SLI_RETR.write(0)
        self.rob.set_digital_out(DIG_OUT_CYL_BOT, 1)
        time.sleep(3) 
        get_logger(__name__).log(logging.DEBUG,
            f"retracted safety system")
        

    def move_on_conv_pos(self, crate_height):
        """
        This function contains the movements:
        - move crate on conveyor
        """
        get_logger(__name__).log(logging.INFO,
            f"Starting move on conveyor")
        self.rob.set_digital_out(DIG_OUT_CONV, 0)
        pos = copy.deepcopy(self.place_pos)
        movement = round(crate_height - 0.17,3)
        pos[2] += movement
        self.rob.movel(pos, acc=1, vel=0.05)
        get_logger(__name__).log(logging.INFO,
            f"Completed move on conveyor")
        

    def move_place_crate(self):
        """
        This function contains the movements:
        - tilt foward around the hooks to place the crate
        - move up from the placed crate 
        """
        get_logger(__name__).log(logging.INFO,
            f"Starting placing crate")
        movement = [0.004, 0 , -0.004, 0, -0.25, 0] #move around hooks
        movement = [x*1.25 for x in movement]
        self.rob.movel_tool(movement, acc=1, vel=0.05)
        self.rob.movel([0,0,0.2,0,0,0], acc=1, vel=0.1, relative=True)
        get_logger(__name__).log(logging.INFO,
            f"Completed placing crate")
        

    def turn_conv_on(self):
        """
        This function contains:
        - turing the conveyer on 
        """
        get_logger(__name__).log(logging.DEBUG,
            f"drop off comfirmed, turning on conveyer")
        self.rob.set_digital_out(DIG_OUT_CONV, 1)
        get_logger(__name__).log(logging.DEBUG,
            f"turned conveyer on")
        time.sleep(3)
        self.rob.set_digital_out(DIG_OUT_CONV, 0)

    def stop(self):
        """
        Hard stop robot
        -> Status Stopped -> priority 1 = high, 2 = low
        """
        self.rob.stop()
        self._change_status(Status.Stopped)
        get_logger(__name__).log(logging.WARNING,
                                 "Stopped robot")

    def destack_done(self):
        """
        Soft stop robot
        -> Status Done
        """
        self._change_status(Status.Done)
        #TODO: remove / add functionality

    def _change_status(self, status, data={}):
        self.handler.change_status(status, data)