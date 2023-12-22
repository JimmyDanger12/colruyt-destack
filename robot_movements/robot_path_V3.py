import numpy as nb
import urx
import time

dig_out_cyl_bot = 1
dig_out_cyl_sli = 2
dig_in_dropoff = 3
ana_in_pressR = 4
ana_in_pressL = 5
ana_out_conv = 6

tcp_bar = [0, 0.02, 0.1095, 0, 0, 0]
tcp_plate = [0, -0.125, 0.0895, 0, 0, 0]

pre_pick = []
place_pos = []
pre_place = []
via_pose = []

def get_digital_in(self, nb, wait=False):
    """
    get digital output
    """
    return self.secmon.get_digital_in(nb, wait)

def set_digital_out(self, output, val):
    """
    set digital output. val is a bool
    """
    if val in (True, 1):
        val = "True"
    else:
        val = "False"
    self.send_program('digital_out[%s]=%s' % (output, val))

def get_analog_in(self, nb, wait=False):
    """
    get analog input
    """
    return self.secmon.get_analog_in(nb, wait=wait)

def set_analog_out(self, output, val):
    """
    set analog output, val is a float
    """
    prog = "set_analog_out(%s, %s)" % (output, val)
    self.send_program(prog)

def main():
    robot = urx.Robot("192.168.0.100")
    robot.set_tcp(tcp_bar)

    robot.movej(pre_place, a=1, v=0.5,) #start pos
    #retrieve pick location and oriantation form vision code
    pick_loc = []
    pick_ori = []
    picked_ori = [pick_ori[0]-20, pick_ori[1], pick_ori[2]]
    Crate_size = ()
    time.sleep(3)
    #if no coordinates are retrieved break and alert 
    
    robot.movec(via_pose, pre_pick, a=1, v=0.5, r=0.2, mode=1) #pre pick pos
    time.sleep(1)

    robot.movej([pre_pick[0:2],pick_ori], a=1, v= 0.25) #pre pick pos with pick ori
    time.sleep(1)
    robot.movel([pick_loc[0], pick_loc[1], pick_loc[2]+0.03, pick_ori[0]+10, pick_ori[1], pick_ori[2]], a=1, v= 0.25) #orientated for pre-pickup and above crate
    time.sleep(3)
    robot.movel([pick_loc,  pick_ori[0]+10, pick_ori[1], pick_ori[2]], a=1, v= 0.1) #orientated for pre-pickup and on crate
    time.sleep(3)
    robot.movel([pick_loc,  pick_ori], a=1, v= 0.1) #orientated for pickup and on crate
    #switch to plate tcp, but that fucks up the pick_loc so then we need to add an offset to the pick_loc before tilting
    time.sleep(5)
    robot.movel([pick_loc, picked_ori], a=0.5, v= 0.1) #crate picked in pos
    pressure_value = get_analog_in(ana_in_pressR) + get_analog_in(ana_in_pressL)
    while pressure_value > 800: #only move if crate is picked (pressure on hooks)
        robot.movel([0, 0, 0.03, 0, 0, 0], a=0.5, v= 0.1, relative=True) #move a litte up 
        time.sleep(1)
        robot.movel([pick_loc[0], pre_pick[1], pick_loc[2]+0.03 , picked_ori], a=1, v= 0.25, r=0, t=0, r=0) #move out of carrier 
        time.sleep(2)
        set_digital_out(dig_out_cyl_bot, True)
        time.sleep(5)
        set_digital_out(dig_out_cyl_sli, True)
        time.sleep(5)
        set_digital_out(dig_out_cyl_bot, False) #support crate with safety system 
        time.sleep(3)
        robot.movel([pre_pick[0:2],picked_ori], a=1, v= 1, t=0, r=0) #move to the pre pick
        time.sleep(3)
        robot.movec([via_pose[0:2],picked_ori], [pre_place[0:2],picked_ori], a=1, v=0.5, r=0, mode=1) #move safely to the pre place 
        set_analog_out(ana_out_conv, 0) #stop conveyer belt
        time.sleep(5)
        set_digital_out(dig_out_cyl_bot, True)
        time.sleep(3)
        set_digital_out(dig_out_cyl_sli, False)
        time.sleep(3)
        set_digital_out(dig_out_cyl_bot, False) #retract safety system
        time.sleep(5)
        robot.movel([place_pos[0], place_pos[1], place_pos[2]+Crate_size, picked_ori], a=1, v= 1, r=0, t=0, r=0) #hold crate on top of conveyer
        time.sleep(5)
        break #stop reading pressure sensors
    #else:
        #break and alert

    robot.movel([place_pos], a=1, v= 1, r=0, t=0, r=0) #place crate on conveyer
    time.sleep(5)
    robot.movel(pre_place, a=1, v= 1, r=0, t=0, r=0) #EoAT to start pos
   
    dropoff_value = get_digital_in(dig_in_dropoff) #reading if crate is placed
    if dropoff_value:
        set_analog_out(ana_out_conv, 1) #if crate has been placed start conveyer
    #else:
        #break and alert 

    return

