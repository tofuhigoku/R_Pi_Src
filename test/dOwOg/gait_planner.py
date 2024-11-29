import numpy as np
import math
from math import pi

# class foot:
#     default_pos = np.zeros((4,4), dtype= float)
#     desired_pos = np.zeros((4,4), dtype= float)
#     init_flag = True
#     def __init__(self, default_pos_list = [0 , 0 , -100, 1 ]):
#         if (default_pos_list[2]>=0):
#             print("INVALID default foots position. z must be < 0 which is {}".format(default_pos_list[2]))
#             self.init_flag = False
#             default_pos_list = [0 , 0 , -100, 1 ]
#         self.default_pos = np.vstack([default_pos_list, default_pos_list, default_pos_list, default_pos_list]).T
#         if (default_pos_list[0]!=0):
#             self.default_pos[0,1] = self.default_pos[0,1]*-1
#             self.default_pos[0,3] = self.default_pos[0,3]*-1
#         if (default_pos_list[1]!=0):
#             self.default_pos[1,1] = self.default_pos[1,1]*-1
#             self.default_pos[1,3] = self.default_pos[1,3]*-1
        
#         self.desired_pos = self.default_pos
        

class pattern_planner:
    stance_phase = np.ones((4,1), dtype= int)
    trot_pattern = np.array([[0,1,0,1],[1,0,1,0]], dtype = int).transpose()
    crawl_pattern = np.array([[0,1,1,1],[1,1,0,1],[1,0,1,1],[1,1,1,0]], dtype = int).transpose()
    trot_T = 0.5
    crawl_T = 0.5
    mode = "stand"
    current_phase = None
 
    def __init__(self, mode="stand", trot_T = 0.5, crawl_T=0.5):
        self.current_phase = self.stance_phase
        self.trot_T = trot_T
        self.crawl_T = crawl_T
        self.mode = mode

    def switch_mode(self, target_mode = "stand"):
        self.current_phase = self.stance_phase
        self.t = 0
        if(target_mode == "stand"):
            self.mode = "stand"
        elif(target_mode == "trot"):
            self.mode = "trot"
        elif(target_mode == "crawl"):
            self.mode = "crawl"
        else:
            self.mode = "stand"
            


    def current_trot_phase(self, t):
        if(t%self.trot_T<(0.5*self.trot_T)):
            self.current_phase = self.trot_pattern[:,0:1]
        else:
            self.current_phase = self.trot_pattern[:,1:2]
        
    def current_crawl_phase(self, t):
        if(t%self.trot_T<(0.25*self.trot_T)):
            self.current_phase = self.crawl_pattern[:,0:1]
        elif(t%self.trot_T<(0.5*self.trot_T)):
            self.current_phase = self.crawl_pattern[:,1:2]
        elif(t%self.trot_T<(0.75*self.trot_T)):
            self.current_phase = self.crawl_pattern[:,2:3]
        else:
            self.current_phase = self.crawl_pattern[:,3:4]

class trajectory_planner:
    default_pos = np.zeros((4,4), dtype= float).transpose()
    desired_pos = np.zeros((4,4), dtype= float).transpose()
    init_flag = True
    _pattern_planner = pattern_planner()
    d_xyzyaw = []

    def __init__(self, default_pos_list = [0 , 0 , -180, 1 ], mode="stand", trot_T = 0.5, crawl_T=0.5):
        self._pattern_planner.current_phase = self._pattern_planner.stance_phase
        self._pattern_planner.trot_T = trot_T
        self._pattern_planner.crawl_T = crawl_T
        self._pattern_planner.mode = mode
        
        if (default_pos_list[2]>=0):
            print("INVALID default foots position. z must be < 0 which is {}".format(default_pos_list[2]))
            self.init_flag = False
            default_pos_list = [0 , 0 , -100, 1 ]
        self.default_pos = np.vstack([default_pos_list, default_pos_list, default_pos_list, default_pos_list]).T
        if (default_pos_list[0]!=0):
            self.default_pos[0,1] = self.default_pos[0,1]*-1
            self.default_pos[0,3] = self.default_pos[0,3]*-1
        if (default_pos_list[1]!=0):
            self.default_pos[1,1] = self.default_pos[1,1]*-1
            self.default_pos[1,3] = self.default_pos[1,3]*-1
        
        self.desired_pos = self.default_pos.copy()
    
    def planning(self, t, d_xyzyaw = None):
        if (d_xyzyaw is not None):
            self.d_xyzyaw = d_xyzyaw

        if (self.init_flag == True):
            if(self._pattern_planner.mode == "trot"):
                self._pattern_planner.current_trot_phase(t)
                # print("trot")
                __tmp_arr = np.copy(self.default_pos)
                self.desired_pos[0,0] = ((self.d_xyzyaw[0])*(math.sin((t/(0.25*self._pattern_planner.trot_T ))*pi*0.5+3*pi/2)*0.5-0.5) + __tmp_arr[0,0])
                self.desired_pos[1,0] = ((self.d_xyzyaw[1])*(math.sin((t/(0.25*self._pattern_planner.trot_T ))*pi*0.5+3*pi/2)*0.5-0.5) + __tmp_arr[1,0])
                self.desired_pos[2,0] = ((self.d_xyzyaw[2])*(math.sin((t/(0.5*self._pattern_planner.trot_T ))*pi))*(1-self._pattern_planner.current_phase[0]) + __tmp_arr[2,0])

                self.desired_pos[0,1] = ((self.d_xyzyaw[0])*(math.sin((t/(0.25*self._pattern_planner.trot_T ))*pi*0.5+1*pi/2)*0.5-0.5) + __tmp_arr[0,1])
                self.desired_pos[1,1] = ((self.d_xyzyaw[1])*(math.sin((t/(0.25*self._pattern_planner.trot_T ))*pi*0.5+1*pi/2)*0.5-0.5) + __tmp_arr[1,1])
                # self.desired_pos[2,1] = ((self.d_xyzyaw[2])*(math.sin((t/(0.5*self._pattern_planner.trot_T ))*pi))*(1-self._pattern_planner.current_phase[1]) + __tmp_arr[2,1])
                self.desired_pos[2,1] = ((self.d_xyzyaw[2])*(math.sin((t/(0.5*self._pattern_planner.trot_T ))*pi))*(-1)*(1-self._pattern_planner.current_phase[1]) + __tmp_arr[2,1])


                self.desired_pos[:,2] = self.desired_pos[:,0].copy()

                self.desired_pos[:,3] = self.desired_pos[:,1].copy()

            elif(self._pattern_planner.mode == "crawl"):
                self._pattern_planner.current_crawl_phase(t)
                print("crawl")

            elif(self._pattern_planner.mode == "stand"):
                print("stand")
                
            else:
                self._pattern_planner.mode == "stand"
                print("stand")