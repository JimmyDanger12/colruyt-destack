import urx
#from vision.vision_client import NoPickUpCrateException, NoDetectedCratesException, VisionClient
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
    def __init__(self, ip, tcp, handler):
        self.handler = handler
        self.ip = ip
        #self.vision_client = VisionClient() #maybe move to connect method
        self.rob = None
        self.tcp = tcp
        self.robot_position = None #use? or always getl in movement functions
        self.pick_loc = None

        self.tcp_hooks = [0, 0.05583, 0.0915, 0, 0, 0]
        self.tcp_bar = [0.02, 0, 0.1095, 0, 0, 0]
        self.tcp_plate = [-0.125, 0, 0.0895, 0, 0, 0]

        self.tcp_z_high = [0,0,0.3,0,0,0]
        self.tcp_z_neg = [0,0,-0.3,0,0,0]
        self.tcp_zero = [0,0,0,0,0,0]
        self.tcp_y_high = [0,0.3,0,0,0,0]
        self.tcp_y_neg = [0,-0.3,0,0,0,0]
        self.tcp_x_high = [0.3,0,0,0,0,0]
        self.tcp_x_neg = [-0.3,0,0,0,0,0]

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
            self.rob.set_tcp(self.tcp_bar)
            self.move_start_pos() 
            pick_coords, pick_loc, pick_ori, picked_ori, crate_height = self.retrieve_pick_pos()
            if pick_coords == []:
                break #alert and log
            self.move_pre_pick_pos() 
            self.move_pre_picked_pos(pick_loc, pick_ori) 
            self.move_picked_pos(pick_loc, pick_ori) 
            self.move_out_carrier(pick_loc, picked_ori) 
            #self.depl_safety_syst()
            self.move_pre_place_pos(picked_ori) 
            #self.retr_safety_syst()
            self.move_on_conv_pos(crate_height, picked_ori) 
            self.move_place_crate() 
            """dropoff_value = self.rob.get_digital_in(DIG_IN_DROPOFF)
            if dropoff_value == True:
                self.turn_conv_on()
            else: 
                break #add alert"""
            break
        return

    def move_start_pos(self):
        get_logger(__name__).log(logging.DEBUG,
            f"Starting move to start pos")
        self.rob.set_tcp(self.tcp_bar)
        get_logger(__name__).log(logging.DEBUG,
            f"set tcp")
        self.rob.movel(self.start_pos, acc=1, vel=0.1)
        get_logger(__name__).log(logging.DEBUG,
            f"Move start pos completed")

    def retrieve_pick_pos(self):
        get_logger(__name__).log(logging.DEBUG,
            f"Retrieval of pick position started")
        """try:
            pick_coords, crate_height = self.vision_client.get_valid_pickup_loc()
        except NoDetectedCratesException:
            get_logger(__name__).log(logging.INFO,
                "No Crates detected")
            #TODO: here leave robot movement
        except NoPickUpCrateException:
            get_logger(__name__).log(logging.INFO,
                "Unpickable crate detected")
            #TODO: here raise to worker"""
        pick_coords = [-0.18016, -1.05289, -0.11338, 1.24, -1.2, 1.24] 
        crate_height = 0.33
        get_logger(__name__).log(logging.DEBUG,
            f"Retreived coords, crate_size from vision {pick_coords}, {crate_height}")
        pick_loc = pick_coords[0:3]
        pick_ori = pick_coords[3:6]
        picked_ori = [pick_ori[0]-20, pick_ori[1], pick_ori[2]]  #TODO: remove
        get_logger(__name__).log(logging.DEBUG,
            f"Retrieval of pick coordinates completed")
        return pick_coords, pick_loc, pick_ori, picked_ori, crate_height

    def move_pre_pick_pos(self):
        get_logger(__name__).log(logging.DEBUG,
            f"starting move to pre pick pos")
        self.rob.movels([self.via_pose,self.pre_pick],
                        acc=1,
                        vel=0.1,
                        radius=0.05)
        #self.rob.movec(self.via_pose, self.pre_pick, acc=1, vel=0.1) #TODO: test angles for clean rotation
        get_logger(__name__).log(logging.DEBUG,
            f"completed move to pre pick pos")
    
    def move_pre_picked_pos(self, pick_loc, pick_ori):
        get_logger(__name__).log(logging.DEBUG,
            f"starting move to pre picked")
        
        self.rob.movel(self.pre_pick,
                   acc=1,
                   vel=0.05)
        self.rob.movel([pick_loc[0], pick_loc[1], pick_loc[2]+0.03] + pick_ori, acc= 1, vel= 0.025)
        self.rob.movel_tool([0, 0, 0, 0, -0.35, 0], acc=1, vel=0.01)
        get_logger(__name__).log(logging.DEBUG,
            f"completed move to pre picked")

    def move_picked_pos(self,pick_loc,pick_ori):
        get_logger(__name__).log(logging.DEBUG,
            f"starting picking")
        self.rob.movel([0, 0, -0.0375, 0, 0, 0], acc=1, vel= 0.01, relative= True)
        
        movement = [-0.005, 0 , 0.005, 0, 0.35, 0] #move around hooks
        self.rob.movel_tool(movement,acc=1,vel=0.01)
        movement = [-0.01,0,-0.05,0,0.35,0] #move around plate
        movement = [x*1.2 for x in movement]
        self.rob.movel_tool(movement)
        get_logger(__name__).log(logging.DEBUG,
            "Move picked_pos completed")

    def move_out_carrier(self, pick_loc, picked_ori):
        get_logger(__name__).log(logging.DEBUG,
            f"staring move out carrier")
        self.rob.movel([0, 0, 0.03, 0, 0, 0], acc=0.5, vel= 0.01, relative=True)
        self.rob.movel([0,0.40,0,0,0,0],acc=0.5,vel=0.025, relative=True)
        """self.rob.movel([pick_loc[0], self.pre_pick[1], pick_loc[2]+0.05] + picked_ori, acc=1, vel= 0.025, wait=False)
        while True: #TODO: add pressure sensor
            pressure_value = self.rob.get_analog_in(ANA_IN_PRESSR) + self.rob.get_analog_in(ANA_IN_PRESSL)
            if pressure_value < 800:
                self.stop()
            if self.rob.get_pose() == [pick_loc[0], self.pre_pick[1], pick_loc[2]+0.05] + picked_ori:
                break"""
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
            f"starting move to pre place")
        self.rob.movel(self.post_pick,
                       acc=1,
                       vel=0.025)
        self.rob.movels([self.post_via_pose,self.pre_place],
                        acc=1,
                        vel=0.05,
                        radius=0.05)
        self.rob.movel(self.pre_place,
                       acc=1,vel=0.025)
        get_logger(__name__).log(logging.DEBUG,
            f"completed move to pre place")

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

    def move_on_conv_pos(self, crate_height, picked_ori):
        get_logger(__name__).log(logging.DEBUG,
            f"starting move on conveyer")
        pos = self.place_pos
        pos[2] += crate_height - 0.185
        self.rob.movel(pos, acc=1, vel= 0.025)
        get_logger(__name__).log(logging.DEBUG,
            f"completed move on conveyer")

    def move_place_crate(self):
        get_logger(__name__).log(logging.DEBUG,
            f"starting move place crate")
        movement = [0.004, 0 , -0.004, 0, -0.25, 0] #move around hooks
        self.rob.movel_tool(movement,acc=1,vel=0.01)
        self.rob.movel([0,0,0.2,0,0,0], acc=1, vel= 0.025, relative=True)
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