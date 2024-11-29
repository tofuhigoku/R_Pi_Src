from bot import bOwOt
from math_ops import math_ops
from gait_planner import pattern_planner, trajectory_planner
from matplotlib import pyplot as plt

import multiprocessing
import threading
import spidev
import serial
import time
import binascii
import random
import struct
import threading
import math
import numpy as np
import sys
# import RPi.GPIO as GPIO
const_ = math_ops()
bot1 = bOwOt("bot1")
bot1.display_legstate()
bot1.display_legcmd()

# x_values = [1,2,3,4]
# y_values = [5,6,7,8]

# plt.plot(x_values, y_values)
# plt.show()
gait_T = 0.5
time_span = [0]
trot_pattern = np.array([[0,1,0,1],[1,0,1,0]], dtype = int).transpose()
stand_pattern = np.ones([4,1], dtype= int)
current_phase = stand_pattern
leg1phase = [1]
leg2phase = [1]
leg3phase = [1]
leg4phase = [1]
phase_span = current_phase
print(phase_span)
pattern_planner_ = pattern_planner()

trajectory_planner_ = trajectory_planner(mode = "trot")

# f1_init = np.array([[0,117.25,-185, 0]]).T
# f1_current_desired = f1_init

f1_trajectory = trajectory_planner_.default_pos[:,0].copy()
f2_trajectory = trajectory_planner_.default_pos[:,1].copy()
f3_trajectory = trajectory_planner_.default_pos[:,2].copy()
f4_trajectory = trajectory_planner_.default_pos[:,3].copy()

# print("f1_trajectory: {}".format(f1_trajectory))
# dx = 50
# dy = 0
# dz = 85
# d_ = np.array([[dx,dy,dz, 0]]).T
def main():
    global phase_span, current_phase, gait_T, time_span, trot_pattern, f1_trajectory, trajectory_planner_, f2_trajectory, f3_trajectory, f4_trajectory
    cur_time = time.time()
    last_time = time.time()
    delta_time = 0.01
    t = 0
    pattern_planner_.switch_mode("trot")
    while True:
        cur_time = time.time()
        if (cur_time - last_time > delta_time):
            t += delta_time
            last_time = cur_time
            time_span.append(t)
            pattern_planner_.current_trot_phase(t)

            # phase_span = np.hstack((phase_span, pattern_planner_.current_phase))

            trajectory_planner_.planning(t, d_xyzyaw = [50,50,50, 1])
            phase_span = np.hstack((phase_span, trajectory_planner_._pattern_planner.current_phase))
            
            # f1_current_desired[0] = dx*(math.sin((t/(0.25*pattern_planner_.trot_T))*const_._PI_2+const_._3PI_2)*0.5-0.5) + 0
            # f1_current_desired[1] = dy*(math.sin((t/(0.25*pattern_planner_.trot_T))*const_._PI_2+const_._3PI_2)*0.5-0.5) + 117.25
            # f1_current_desired[2] = dz*(math.sin((t/(0.5*pattern_planner_.trot_T))*const_._PI_))*(1-pattern_planner_.current_phase[0]) -185
            # print("f1 and current {}\t{}".format(f1_trajectory,trajectory_planner_.desired_pos[:,0]))
            f1_trajectory = np.vstack((f1_trajectory, trajectory_planner_.desired_pos[:,0]))
            f2_trajectory = np.vstack((f2_trajectory, trajectory_planner_.desired_pos[:,1]))
            f3_trajectory = np.vstack((f3_trajectory, trajectory_planner_.desired_pos[:,2]))
            f4_trajectory = np.vstack((f4_trajectory, trajectory_planner_.desired_pos[:,3]))


            if (t>=4):
                break
if __name__ == '__main__':
    main()


fig, (L1_phase, L2_phase,L3_phase,L4_phase) = plt.subplots(4, sharex=True)
fig.suptitle('gait_pattern')
L1_phase.plot(time_span, phase_span[0,:])
L2_phase.plot(time_span, phase_span[1,:])
L3_phase.plot(time_span, phase_span[2,:])
L4_phase.plot(time_span, phase_span[3,:])

fig2, (L1_phase, f1x, f2x, f3x, f4x) = plt.subplots(5, sharex=True)
fig2.suptitle('x compare')
L1_phase.plot(time_span, phase_span[0,:])
f1x.plot(time_span, f1_trajectory[:,0])
f2x.plot(time_span, f2_trajectory[:,0])
f3x.plot(time_span, f3_trajectory[:,0])
f4x.plot(time_span, f4_trajectory[:,0])


fig3, (L1_phase, f1y, f2y, f3y, f4y) = plt.subplots(5, sharex=True)
fig3.suptitle('y compare')
L1_phase.plot(time_span, phase_span[0,:])
f1y.plot(time_span, f1_trajectory[:,1])
f2y.plot(time_span, f2_trajectory[:,1])
f3y.plot(time_span, f3_trajectory[:,1])
f4y.plot(time_span, f4_trajectory[:,1])


fig4, (L1_phase, f1z, f2z, f3z, f4z) = plt.subplots(5, sharex=True)
fig4.suptitle('z compare')
L1_phase.plot(time_span, phase_span[0,:])
f1z.plot(time_span, f1_trajectory[:,2])
f2z.plot(time_span, f2_trajectory[:,2])
f3z.plot(time_span, f3_trajectory[:,2])
f4z.plot(time_span, f4_trajectory[:,2])

fig5, (L1_phase, L2_phase, f1z, f2z) = plt.subplots(4, sharex=True)
fig5.suptitle('z compare 2')
L1_phase.plot(time_span, phase_span[0,:])
L2_phase.plot(time_span, phase_span[1,:])

f1z.plot(time_span, f1_trajectory[:,2])
f2z.plot(time_span, f2_trajectory[:,2])

plt.show()
print("hello controller")