import numpy as np 
import os
import spidev
import serial
import i2cdevice
import time
import math

class joint_state:
    pos = 0
    vel = 0
    torque = 0
    Id = 0
    Iq = 0

class joint_offset:
    pos = 0

class joint_cmd:
    pos = 0
    vel = 0
    torque = 0
    Kd = 0
    Kq = 0

class joint:
    state = joint_state()
    cmd = joint_cmd()
    offset = joint_offset()

class leg_i:
    abd_joint = joint()
    hip_joint = joint()
    knee_joint = joint()
    foot_pos = np.zeros([4,1], dtype= float)
    foot_cmd = np.zeros([4,1], dtype= float)


class bOwOt:
    leg1 = leg_i()
    leg2 = leg_i()
    leg3 = leg_i()
    leg4 = leg_i()
    def __init__(self, name): 
        self.name = name 
        print("class " +str(self.name) + " was created!\n" )
    def display_legstate(self):
        print("{}.leg1_state.pos: \tabd={}\thip:{}\tknee{}".format(self.name, self.leg1.abd_joint.state.pos, self.leg1.hip_joint.state.pos,self.leg1.knee_joint.state.pos) )
        print("{}.leg2_state.pos: \tabd={}\thip:{}\tknee{}".format(self.name, self.leg2.abd_joint.state.pos, self.leg2.hip_joint.state.pos,self.leg2.knee_joint.state.pos) )
        print("{}.leg3_state.pos: \tabd={}\thip:{}\tknee{}".format(self.name, self.leg3.abd_joint.state.pos, self.leg3.hip_joint.state.pos,self.leg3.knee_joint.state.pos) )
        print("{}.leg4_state.pos: \tabd={}\thip:{}\tknee{}".format(self.name, self.leg4.abd_joint.state.pos, self.leg4.hip_joint.state.pos,self.leg4.knee_joint.state.pos) )

        print("{}.leg1_state.vel: \tabd={}\thip:{}\tknee{}".format(self.name, self.leg1.abd_joint.state.vel, self.leg1.hip_joint.state.vel,self.leg1.knee_joint.state.vel) )
        print("{}.leg2_state.vel: \tabd={}\thip:{}\tknee{}".format(self.name, self.leg2.abd_joint.state.vel, self.leg2.hip_joint.state.vel,self.leg2.knee_joint.state.vel) )
        print("{}.leg3_state.vel: \tabd={}\thip:{}\tknee{}".format(self.name, self.leg3.abd_joint.state.vel, self.leg3.hip_joint.state.vel,self.leg3.knee_joint.state.vel) )
        print("{}.leg4_state.vel: \tabd={}\thip:{}\tknee{}".format(self.name, self.leg4.abd_joint.state.vel, self.leg4.hip_joint.state.vel,self.leg4.knee_joint.state.vel) )
    def display_legcmd(self):
        print("{}.leg1_cmd.pos: \tabd={}\thip:{}\tknee{}".format(self.name, self.leg1.abd_joint.cmd.pos, self.leg1.hip_joint.cmd.pos,self.leg1.knee_joint.cmd.pos) )
        print("{}.leg2_cmd.pos: \tabd={}\thip:{}\tknee{}".format(self.name, self.leg2.abd_joint.cmd.pos, self.leg2.hip_joint.cmd.pos,self.leg2.knee_joint.cmd.pos) )
        print("{}.leg3_cmd.pos: \tabd={}\thip:{}\tknee{}".format(self.name, self.leg3.abd_joint.cmd.pos, self.leg3.hip_joint.cmd.pos,self.leg3.knee_joint.cmd.pos) )
        print("{}.leg4_cmd.pos: \tabd={}\thip:{}\tknee{}".format(self.name, self.leg4.abd_joint.cmd.pos, self.leg4.hip_joint.cmd.pos,self.leg4.knee_joint.cmd.pos) )

        print("{}.leg1_cmd.vel: \tabd={}\thip:{}\tknee{}".format(self.name, self.leg1.abd_joint.cmd.vel, self.leg1.hip_joint.cmd.vel,self.leg1.knee_joint.cmd.vel) )
        print("{}.leg2_cmd.vel: \tabd={}\thip:{}\tknee{}".format(self.name, self.leg2.abd_joint.cmd.vel, self.leg2.hip_joint.cmd.vel,self.leg2.knee_joint.cmd.vel) )
        print("{}.leg3_cmd.vel: \tabd={}\thip:{}\tknee{}".format(self.name, self.leg3.abd_joint.cmd.vel, self.leg3.hip_joint.cmd.vel,self.leg3.knee_joint.cmd.vel) )
        print("{}.leg4_cmd.vel: \tabd={}\thip:{}\tknee{}".format(self.name, self.leg4.abd_joint.cmd.vel, self.leg4.hip_joint.cmd.vel,self.leg4.knee_joint.cmd.vel) )

# newbot = bOwOt("jj")





# print("hello python bot!")