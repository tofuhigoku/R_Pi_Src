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
import RPi.GPIO as GPIO

_PI_ = 3.14159265;
_2PI_= 6.2831853
start_buzzer_time = time.time()
buzzer_pin = 26
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM);
GPIO.setup(buzzer_pin, GPIO.OUT, initial=GPIO.LOW);
while True:
    dt = time.time()  - start_buzzer_time;
    GPIO.output(buzzer_pin, GPIO.HIGH);
    #print('buzzing')
    #time.sleep(0.0002);
    #GPIO.output(buzzer_pin, GPIO.LOW);
    #print('done buzzing')
    #time.sleep(0.000002);
    if(dt>=1):
        GPIO.output(buzzer_pin, GPIO.LOW);
        break;
time.sleep(5);
spi0_rx=[]
spi1_rx=[]
spi0_tx=[]
spi1_tx=[]
RC_data = []
decode_uart = []
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout =  0.001)
ser.flush()
uart=[0.0]*33

def pack_float(my_float: float) -> bytes:
    byteobj = struct.pack('<f', my_float);
    return byteobj
        
def unpack_float(my_bytes: bytes) -> float:
    unpacked_float = struct.unpack_from('<f',my_bytes)[0];
    rounded = round(unpacked_float, ndigits=7);
    return rounded

def unpack_float_(b1, b2, b3, b4):
    my_bytes = bytes([b1, b2, b3, b4]);
    unpacked_float = struct.unpack_from('<f',my_bytes)[0];
    rounded = round(unpacked_float, ndigits=7);
    return rounded

def unpack_float__(_4bytes_list):
    my_bytes = bytes(_4bytes_list);
    unpacked_float = struct.unpack_from('<f',my_bytes)[0];
    rounded = round(unpacked_float, ndigits=7);
    return rounded
    
def decode_spidata(spi_rx_buffer):
    float_data = [];
    _digit=0    
    try:
        for i in range(int(len(spi_rx_buffer)/4)):
            #4*i+0,+1,+2,+3
            _digit = 4*i;
            f = unpack_float__(spi_rx_buffer[_digit:_digit+4])
            float_data.append(f)
        return float_data
    except:
        return 0

    
def encode_spicmd(_float_list_data):
    bytes_data_list =[];
    try:
        for i in _float_list_data:
            _4_bytes_float = pack_float(i);
            bytes_data_list.append(_4_bytes_float[0]);
            bytes_data_list.append(_4_bytes_float[1]);
            bytes_data_list.append(_4_bytes_float[2]);
            bytes_data_list.append(_4_bytes_float[3]);
        return bytes_data_list
    except:
        return 0 

def checksum_spi(spi_data_):
    checksum = 0;
    for i in range(int(len(spi_data_)/4)):
        __digit = 4*i;
        __uint32_bit_data = (spi_data_[__digit+0]<<24)|(spi_data_[__digit+1]<<16)|(spi_data_[__digit+2]<<8)|(spi_data_[__digit+3])
        checksum = checksum^__uint32_bit_data
    return checksum

def convert_angle(zero_2_2pi_angle):
    #return math.remainder(zero_2_2pi_angle, 2*math.pi)
    if (zero_2_2pi_angle>_PI_):
        return zero_2_2pi_angle-_2PI_;
    elif (zero_2_2pi_angle<-_PI_):
        return zero_2_2pi_angle+_2PI_;
    else:
        return zero_2_2pi_angle;

def convert_angle3(zero_2_2pi_angle_list3):
    #return math.remainder(zero_2_2pi_angle, 2*math.pi)
    return_list = [0,0,0];
    for i in range(3):
        if(zero_2_2pi_angle_list3[i]>_PI_):
            return_list[i] = zero_2_2pi_angle_list3[i]-_2PI_;
        elif (zero_2_2pi_angle_list3[i]<-_PI_):
            return_list[i] = zero_2_2pi_angle_list3[i]+_2PI_;
        else:
            return_list[i] = zero_2_2pi_angle_list3[i];
            
    return return_list;
############# forward kinematic
def forward_kinematic1(theta_m):
    t11_m = theta_m[0];
    t12_m = theta_m[1];
    t13_m = theta_m[2];
    
    T_B_11  = np.array([[0,0,-1,234],[0,1,0,70],[1,0,0,0],[0,0,0,1]],dtype =float)
    T_h1_11 = np.array([[0,0,-1,0],[0,1,0,0],[1,0,0,0],[0,0,0,1]],dtype =float)
    T_11_12 = np.array([[math.cos(t11_m),0,-math.sin(t11_m),0],
                        [math.sin(t11_m),0, math.cos(t11_m),0],
                        [0,-1,0,0],
                        [0,0,0,1]],dtype =float)
    T_12_13 = np.array([[math.cos(t12_m),-math.sin(t12_m),0,128*math.cos(t12_m)],
                        [math.sin(t12_m), math.cos(t12_m),0,128*math.sin(t12_m)],
                        [0,0,1,117.25],
                        [0,0,0,1]],dtype =float)
    T_13_14 = np.array([[math.cos(t13_m),-math.sin(t13_m),0,157.5*math.cos(t13_m)],
                        [math.sin(t13_m), math.cos(t13_m),0,157.5*math.sin(t13_m)],
                        [0,0,1,0],
                        [0,0,0,1]],dtype =float)
    T_11_14 = T_11_12.dot(T_12_13).dot(T_13_14);
    T_h1_14 = T_h1_11.dot(T_11_14);
    xyz__1 = T_h1_14[0:3,3];
    return xyz__1

def forward_kinematic2(theta_m):
    t21_m = theta_m[0];
    t22_m = theta_m[1];
    t23_m = theta_m[2];
    
    T_B_21  = np.array([[0,0,1,234],[0,-1,0,-70],[1,0,0,0],[0,0,0,1]],dtype =float)
    T_h2_21 = np.array([[0,0,1,0],[0,-1,0,0],[1,0,0,0],[0,0,0,1]],dtype =float)
    T_21_22 = np.array([[math.cos(t21_m),0,-math.sin(t21_m),0],
                        [math.sin(t21_m),0, math.cos(t21_m),0],
                        [0,-1,0,0],
                        [0,0,0,1]],dtype =float)
    T_22_23 = np.array([[math.cos(t22_m),-math.sin(t22_m),0,128*math.cos(t22_m)],
                        [math.sin(t22_m), math.cos(t22_m),0,128*math.sin(t22_m)],
                        [0,0,1,117.25],
                        [0,0,0,1]],dtype =float)
    T_23_24 = np.array([[math.cos(t23_m),-math.sin(t23_m),0,157.5*math.cos(t23_m)],
                        [math.sin(t23_m), math.cos(t23_m),0,157.5*math.sin(t23_m)],
                        [0,0,1,0],
                        [0,0,0,1]],dtype =float)
    T_21_24 = T_21_22.dot(T_22_23).dot(T_23_24);
    T_h2_24 = T_h2_21.dot(T_21_24);
    xyz__2 = T_h2_24[0:3,3];
    return xyz__2

    
############# end of forward kinematic

############# inverse kinematic
def inverse_kinematic1(xyz):
    x_ = xyz[0]
    y_ = xyz[1]
    z_ = xyz[2]
    xyz_sqare = x_*x_ + y_*y_ + z_*z_;
    OA = 117.25;	#OA = 112.5;
    AB = 128.0;		#AB = 130.0;
    BC = 157.5;		#BC = 170.0;
    OD = abs(math.sqrt(y_*y_ + z_*z_));
    alpha_ = math.acos(abs(y_)/OD);
    beta_ = math.acos(OA/OD);
    gamma_  = _PI_/2 - beta_;
    
    if(y_==0):
        q1 = _PI_/2 - abs(alpha_ - gamma_);
        t11 = q1;
    else:
        q1 = _PI_/2 - abs(alpha_ + np.sign(y_)*gamma_);
        t11 = -1*np.sign(y_)*q1;
        
    PHI = math.acos(abs(x_/math.sqrt(xyz_sqare - OA*OA)))
    phi = math.acos((AB*AB + xyz_sqare - OA*OA - BC*BC)/abs(2*AB*math.sqrt(xyz_sqare - OA*OA)))
    if(x_ >0):
        q2 = _PI_/2 - abs(PHI) - abs(phi);
    else:
        q2 = -_PI_/2 + abs(PHI) - abs(phi);
    t12 = _PI_- q2;
    
    q3 = math.acos(-(AB*AB + BC*BC - (xyz_sqare-OA*OA))/abs(2*AB*BC));
    t13 = 2*_PI_ - q3;
    return [t11, t12, t13]


def inverse_kinematic2(xyz):
    x_ = xyz[0]
    y_ = xyz[1]
    z_ = xyz[2]
    xyz_sqare = x_*x_ + y_*y_ + z_*z_;
    OA = 117.25;	#OA = 112.5;
    AB = 128.0;		#AB = 130.0;
    BC = 157.5;		#BC = 170.0;
    OD = abs(math.sqrt(y_*y_ + z_*z_));
    alpha_ = math.acos(abs(y_)/OD);
    beta_ = math.acos(OA/OD);
    gamma_  = _PI_/2 - beta_;
    
    if(y_==0):
        q1 = _PI_/2 - abs(alpha_ - gamma_);
        t21 = q1;
    else:
        q1 = _PI_/2 - abs(alpha_ - np.sign(y_)*gamma_);
        t21 = np.sign(y_)*q1;
        
    PHI = math.acos(abs(x_/math.sqrt(xyz_sqare - OA*OA)))
    phi = math.acos((AB*AB + xyz_sqare - OA*OA - BC*BC)/abs(2*AB*math.sqrt(xyz_sqare - OA*OA)))
    if(x_ >0):
        q2 = _PI_/2 - abs(PHI) - abs(phi);
    else:
        q2 = -_PI_/2 + abs(PHI) - abs(phi);
    t22 = _PI_ + q2;
    
    q3 = math.acos(-(AB*AB + BC*BC - (xyz_sqare-OA*OA))/abs(2*AB*BC));
    t23 = q3;
    return [t21, t22, t23]

############# end of inverse kinematic

############# crawl_planner
def crawl_planner(start_time, T_gait_cycle, dx, dy, dz, z, yaw, gait_):
    start_time_ = start_time;
    while True:
        crawl_dt = time.time() - start_time;
        crawl_dt_in_T = crawl_dt%T_gait_cycle;
        if(dt<T_gait_cycle): #soft start gait
            pass
        else:
            if (gait_ != 'crawl'):
                pass
            else:
                crawl_dt_in_T

def trot_planer(start_time, T_gait_cycle,dx, dy, dz, z, yaw, gaitmode):
    start_time_ = start_time;
    leg1 =[0, 117.25,z]
    leg2 =[0,-117.25,z];
    leg3=leg1;
    leg4 =leg2;
    leg_contact_cycle1 = [0 , 1, 1, 0];  # leg1 n 4 on air, leg2 n 3 landing
    leg_contact_cycle2 = [1 , 0, 0, 1];
    n = 0;
    while (gaitmode == 'trot'):
        print(gaitmode);
        trot_dt = time.time() - start_time;
        trot_dt_in_T = trot_dt%T_gait_cycle;
        if(dx == 0):
            n = 0;
            # dx, dy =0 => trot at recent position
            leg1 = [0, 117.25,z]
            leg2 = [0, -117.25,z]
            leg3 = leg1
            leg4 = leg2
            if(trot_dt_in_T<T_gait_cycle/2):
                leg1[2] = -z + dz*math.sin(trot_dt_in_T*math.pi/(T_gait_cycle/2));
                leg4[2] = leg1[2];
                leg2[2] = -z +dz*0;
                leg3[2] = leg2[2];
                leg_contact_cycle1
            else:
                leg1[2] = -z +dz*0;
                leg4[2] = leg1[2];
                leg2[2] = -z + dz*math.sin((trot_dt_in_T-T_gait_cycle/2)*math.pi/(T_gait_cycle/2));
                leg3[2] = leg2[2];
                leg_contact_cycle2
        else:
            pass

    
############# end of crawl_planner
        
############# standing pose
def standing_pose(z, dz, roll, pitch, yaw):
    z = abs(z) + dz;
    hip_pos1  =  np.array([[233],[70],[0]], dtype = float)
    hip_pos2  =  np.array([[233],[-70],[0]], dtype = float)
    hip_pos3  =  np.array([[-233],[70],[0]], dtype = float)
    hip_pos4  =  np.array([[-233],[-70],[0]], dtype = float)
    
    foot_pos1 =  np.array([[0],[117.25],[-z]], dtype = float)
    foot_pos2 =  np.array([[0],[-117.25],[-z]], dtype = float)
    foot_pos3 =  np.array([[0],[117.25],[-z]], dtype = float)
    foot_pos4 =  np.array([[0],[-117.25],[-z]], dtype = float)
    
    foot_pos1 = foot_pos1 + hip_pos1;
    foot_pos1 = np.vstack((foot_pos1,np.array([1],dtype=float)))
    foot_pos2 = foot_pos2 + hip_pos2;
    foot_pos2 = np.vstack((foot_pos2,np.array([1],dtype=float)))
    foot_pos3 = foot_pos3 + hip_pos3;
    foot_pos3 = np.vstack((foot_pos3,np.array([1],dtype=float)))
    foot_pos4 = foot_pos4 + hip_pos4;
    foot_pos4 = np.vstack((foot_pos4,np.array([1],dtype=float)))
    
    R_A_B = np.array([[math.cos(yaw)*math.cos(pitch),
                       math.cos(yaw)*math.sin(pitch)*math.sin(roll)-math.sin(yaw)*math.cos(roll),
                       math.sin(yaw)*math.sin(roll)+math.cos(yaw)*math.sin(pitch)*math.cos(roll)],
                      [math.sin(yaw)*math.cos(pitch),
                       math.cos(yaw)*math.cos(roll)+math.sin(yaw)*math.sin(pitch)*math.sin(roll),
                       math.sin(yaw)*math.sin(pitch)*math.cos(roll)-math.cos(yaw)*math.sin(roll)],
                      [-math.sin(pitch),
                       math.cos(pitch)*math.sin(roll),
                       math.cos(pitch)*math.cos(roll)]],dtype=float);
    #zv = np.array([0,0,0],dtype = float)
    #zh = np.array([[0],[0],[0],[1]], dtype = float);
    #T_A_B = np.hstack((np.vstack((R_A_B,zv)),zh));
    
    new_leg_hip1_pos = R_A_B.dot(hip_pos1);
    new_leg_hip2_pos = R_A_B.dot(hip_pos2);
    new_leg_hip3_pos = R_A_B.dot(hip_pos3);
    new_leg_hip4_pos = R_A_B.dot(hip_pos4);
    
    zz = np.array([0,0,0,1],dtype = float)    
    T_newhip1_to_bodyframe = np.vstack((np.hstack((R_A_B,new_leg_hip1_pos)),zz));
    T_newhip2_to_bodyframe = np.vstack((np.hstack((R_A_B,new_leg_hip2_pos)),zz));
    T_newhip3_to_bodyframe = np.vstack((np.hstack((R_A_B,new_leg_hip3_pos)),zz));
    T_newhip4_to_bodyframe = np.vstack((np.hstack((R_A_B,new_leg_hip4_pos)),zz));
    
    try:
        rotated_foot_pos1 = np.linalg.inv(T_newhip1_to_bodyframe).dot(foot_pos1)
        rotated_foot_pos2 = np.linalg.inv(T_newhip2_to_bodyframe).dot(foot_pos2)
        rotated_foot_pos3 = np.linalg.inv(T_newhip3_to_bodyframe).dot(foot_pos3)
        rotated_foot_pos4 = np.linalg.inv(T_newhip4_to_bodyframe).dot(foot_pos4)
        #print('f1:',rotated_foot_pos1,'  f2:',rotated_foot_pos2,'  f3:',rotated_foot_pos3,'  f4:',rotated_foot_pos4)
        return np.hstack((rotated_foot_pos1,rotated_foot_pos2,rotated_foot_pos3,rotated_foot_pos4))


        
    except:
        
        print( "inv matrix_fail");
        return np.hstack(([0,117.25,-185],[0,-117.25,-185],[0,117.25,-185],[0,-117.25,-185]))
        
    #T_21_24 = T_21_22.dot(T_22_23).dot(T_23_24);
    

############# end of standing pose
############# velocity  profile
def create_joint_velocity_profile(current_time, T_, q0, qf):
    q_ = 0;
    qd_ = 0;
    qdd_ = 0;
    
    q0_d = 0;
    q0_dd = 0;
    t_ = current_time;
    T__ = T_;
    dt__ = t_%T__;
    #print(dt__);
    a0 = q0;
    a1 = q0_d;
    a2 = q0_dd/2;
    a3 = 10*(qf-q0)/(T__*T__*T__) - 3*q0_dd/(2*T__)-(6*q0_d)/(T__*T__);
    a4 = (3*q0_dd)/(2*T__*T__)+15*(q0-qf)/(T__*T__*T__*T__)+8*q0_d/(T__*T__*T__);
    a5 = 6*(qf-q0)/(T__*T__*T__*T__*T__)-q0_dd/(2*T__*T__*T__)-3*q0_d/(T__*T__*T__*T__);
    
    q_ =round( a0 +a1*dt__ + a2*dt__*dt__ +a3*dt__*dt__*dt__ +a4*dt__*dt__*dt__*dt__ +a5*dt__*dt__*dt__*dt__*dt__, 4);
    qd_ =round( a1 + 2*a2*dt__ + 3*a3*dt__*dt__ + 4*a4*dt__*dt__*dt__ + 5*a5*dt__*dt__*dt__*dt__, 4);
    qdd_ =round( 2*a2 + 6*a3*dt__ + 12*a4*dt__*dt__ + 20*a5*dt__*dt__*dt__,4);
    return [q_, qd_, qdd_]
    
def create_joint_velocity_profile2(current_time, T_, q0, qf):
    q_ = 0;
    qd_ = 0;
    qdd_ = 0;
    
    q0_d = 0;
    q0_dd = 0;
    t_ = current_time;
    T__ = T_;
    dt__ = t_%T__;
    #print(dt__);
    a0 = round(q0,4);
    qf = round(qf,4);
    a1 = q0_d;
    a2 = q0_dd/2;
    a3 = round(10*(qf-q0)/(T__*T__*T__),4);
    a4 = round(15*(q0-qf)/(T__*T__*T__*T__),4);
    a5 = round(6*(qf-q0)/(T__*T__*T__*T__*T__),4);
    
    q_ =round( a0  +a3*dt__*dt__*dt__ +a4*dt__*dt__*dt__*dt__ +a5*dt__*dt__*dt__*dt__*dt__, 4);
    qd_ =round(  3*a3*dt__*dt__ + 4*a4*dt__*dt__*dt__ + 5*a5*dt__*dt__*dt__*dt__, 4);
    qdd_ =round( 6*a3*dt__ + 12*a4*dt__*dt__ + 20*a5*dt__*dt__*dt__,4);
    return [q_, qd_, qdd_]

def create_joint_velocity_profile__(current_time, T_, q0, qf):
    #q_ = 0;
    #qd_ = 0;
    #qdd_ = 0;
    q0_ = np.array([q0]).T;
    q0_d = np.zeros([3,1], dtype = int);
    q0_dd = np.zeros([3,1], dtype = int);
    
    qf_ = np.array([qf]).T;
    if(abs(qf_[0] - q0_[0]) < 0.01):
        qf_[0] =q0_[0];
    if(abs(qf_[1] - q0_[1]) < 0.01):
        qf_[1] =q0_[1];
    if(abs(qf_[2] - q0_[2]) < 0.01):
        qf_[2] =q0_[2];
    t_ = current_time;
    T__ = T_;
    dt__ = t_%T__;
    #print(dt__);
    a0 = q0_;
    a1 = q0_d;
    a2 = q0_dd/2;
    a3 = 10*(qf_-q0_)/(T__*T__*T__) - 3*q0_dd/(2*T__)-(6*q0_d)/(T__*T__);
    a4 = (3*q0_dd)/(2*T__*T__)+15*(q0_-qf_)/(T__*T__*T__*T__)+8*q0_d/(T__*T__*T__);
    a5 = 6*(qf_-q0_)/(T__*T__*T__*T__*T__)-q0_dd/(2*T__*T__*T__)-3*q0_d/(T__*T__*T__*T__);
    
    q_ = a0 +a1*dt__ + a2*dt__*dt__ +a3*dt__*dt__*dt__ +a4*dt__*dt__*dt__*dt__ +a5*dt__*dt__*dt__*dt__*dt__;
    qd_ = a1 + 2*a2*dt__ + 3*a3*dt__*dt__ + 4*a4*dt__*dt__*dt__ + 5*a5*dt__*dt__*dt__*dt__;
    qdd_ = 2*a2 + 6*a3*dt__ + 12*a4*dt__*dt__ + 20*a5*dt__*dt__*dt__;
    return [q_, qd_, qdd_]
    
        
    
############# end of velocity  profile
    
class tx_cmd_input:
    dx = 0;
    dy = 0;
    dz = 0;
    dyaw = 0;
    droll =0;
    dpitch=0;
    gait_mode = 'stand';
    enable =0;
    standby = 0;
    '''
    ch1 = 0;
    ch2 = 0;
    ch3 = 0;
    ch4 = 0;
    ch5 = 0;
    ch6 = 0;
    ch7 = 0;
    '''
    dx_max = 120; #mm
    dy_max = 100; #mm
    dz_max = 80; #mm
    dyaw_max = 0*_PI_/12; #mm
    droll_max = 0*_PI_/10; #mm
    dpitch_max = 0*_PI_/10; #mm
    
    start_time_trot = 0;
    start_time_crawl = 0;
    start_time_stance = 0;
    
    def get_cmd_data(self, RX_signal_data):
        
        for i in range(4):
            if(abs(RX_signal_data[i]) > 120):
                RX_signal_data[i] = 0.0;
        '''
        self.ch1 = RX_signal_data[0];
        self.ch2 = RX_signal_data[1];
        self.ch3 = RX_signal_data[2];
        self.ch4 = RX_signal_data[3];
        self.ch5 = RX_signal_data[4];
        self.ch6 = RX_signal_data[5];
        self.ch7 = RX_signal_data[6];
        '''
        self.dy = int(RX_signal_data[0]/10)/10*self.dy_max;
        self.dx = int(RX_signal_data[1]/10)/10*self.dx_max;
        self.dz = (RX_signal_data[2]/10)/10*self.dz_max;
        
        self.dyaw = -1*int(RX_signal_data[3]/10)/10*self.dyaw_max;
        self.droll = int(RX_signal_data[0]/10)/10*self.droll_max;
        self.dpitch = int(RX_signal_data[1]/10)/10*self.dpitch_max;
        
        if(RX_signal_data[4]==0xAA):
            self.gait_mode = 'trot';
        elif(RX_signal_data[4]==0xCC):
            self.gait_mode = 'crawl';
        elif(RX_signal_data[4]==0xBB):
            self.gait_mode = 'stand';
        
        else:
            self.gait_mode = 'stand';
            
            
            
        if(RX_signal_data[5]==0xAA):
            self.enable = 0x1;
        elif (RX_signal_data[5]==0XEE):
            self.enable = 0xf;
            
        elif(RX_signal_data[5]==0xDD):
            self.enable = 0x0;
            
        else:
            self.enable = 0x0;
            
            
        if(RX_signal_data[6]==0XEE):
            self.standby = 1;
        else:
            self.standby = 0;
        '''
        if(RX_signal_data[4]>50 and RX_signal_data[4]<150):
            self.gait_mode = 'crawl';
        elif(RX_signal_data[4]<-50 and RX_signal_data[4]>-150):
            self.gait_mode = 'trot';
        else:
            self.gait_mode = 'stand';
            
            
        if(RX_signal_data[5]<-80 and self.enable != 0xf):
            self.enable = 0x0;
        elif (RX_signal_data[5]>80 and self.enable != 0x0 ):
            self.enable = 0xf;
        else:
            self.enable = 0x1;
            
            
        if(RX_signal_data[6]>80):
            self.standby = 1;
        else:
            self.standby = 0;
        '''
        
    def print_cmd_data(self):
        print('dx:',self.dx,'  dy:',self.dy,'  dz:',self.dz,
              '  dyaw:',self.dyaw,'  droll:',self.droll,'  dpitch:',self.dpitch,
              '  gait:', self.gait_mode,' enable:',self.enable,'  sb:',self.standby)
        #print('ch1:',self.ch1,'  ch2:',self.ch2,'  ch3:',self.ch3,'  ch4:',self.ch4,'  ch5:',self.ch5,'  ch6:',self.ch6,'  ch7:',self.ch7);
        
        
    
    
    
    
    
class leg_state:
    x = 0;
    y = 0;
    z = 0;
    def __init__(self):
        self.a =self.abad();
        self.h = self.hip();
        self.k =  self.knee();
        
    class abad:
        def __init__(self):
            self.p_init = 0;
            self.p_offset = 0.0
            self.p = 0.0;
            self.v = 0.0;
            self.t = 0.0;
            self.GR = 1;
            
    class hip:        
        def __init__(self):
            self.p_init = 0;
            self.p_offset = 0.0
            self.p =0.0;
            self.v =0.0;
            self.t =0.0;
            self.GR = 1;
            
    class knee:            
        def __init__(self):
            self.p_init = 0;
            self.p_offset = 0.0
            self.p=0.0;
            self.v=0.0;
            self.t=0.0;
            self.GR = 1.25;
            
class leg_cmd:
    x = 0;
    y = 0;
    z = 0;
    
    enable = 0;
    kp_abad =0.0;
    kp_hip = 0.0;
    kp_knee =0.0;
    
    kd_abad =0.0;
    kd_hip = 0.0;
    kd_knee =0.0;
    def __init__(self):
        self.a =self.abad();
        self.h = self.hip();
        self.k =  self.knee();
        
    class abad:
        def __init__(self):
            self.p_des = 0.0;
            self.p_last = 0.0;
            self.v_des = 0.0;
            self.tff = 0.0;
            self.GR = 1;
            
    class hip:        
        def __init__(self):
            self.p_des =0.0;
            self.p_last = 0.0;
            self.v_des =0.0;
            self.tff =0.0;
            self.GR = 1;
            
    class knee:            
        def __init__(self):
            self.p_des =0.0;
            self.p_last = 0.0;
            self.v_des =0.0;
            self.tff=0.0;
            self.GR = 1.25;

TX_cmd = tx_cmd_input();
            
leg1_state = leg_state();
leg2_state = leg_state();
leg3_state = leg_state();
leg4_state = leg_state();

leg1_state.a.p_init = 0;
leg1_state.h.p_init = (254.072*_PI_/180)*0 + -1*_PI_;
leg1_state.k.p_init = (145.5117*_PI_/180)*0 + 0;

leg3_state.a.p_init = 0;
leg3_state.h.p_init = -1*_PI_;
leg3_state.k.p_init = leg1_state.k.p_init;

leg2_state.a.p_init = 0;
leg2_state.h.p_init = (-1*leg1_state.h.p_init)*0 +_PI_;
leg2_state.k.p_init = -1*leg1_state.k.p_init*0 + 0;

leg4_state.a.p_init = 0;
leg4_state.h.p_init = (-1*leg1_state.h.p_init)*0 + _PI_;
leg4_state.k.p_init = -1*leg1_state.k.p_init;

leg1_cmd = leg_cmd();
leg2_cmd = leg_cmd();
leg3_cmd = leg_cmd();
leg4_cmd = leg_cmd();
'''
leg1_cmd.a.p_des =0.2;
leg1_cmd.a.v_des = 0;
leg1_cmd.a.tff=0;

leg1_cmd.h.p_des = 0.2;
leg1_cmd.h.v_des = 0;
leg1_cmd.h.tff=0;

leg1_cmd.k.p_des = 0.2;
leg1_cmd.k.v_des = 0.0;
leg1_cmd.k.tff=0.0;

leg2_cmd.a.p_des = -0.2;
leg2_cmd.a.v_des = 0.0;
leg2_cmd.a.tff=0;

leg2_cmd.h.p_des = -0.2;
leg2_cmd.h.v_des = 0;
leg2_cmd.h.tff=0;

leg2_cmd.k.p_des = -0.2;
leg2_cmd.k.v_des = 0;
leg2_cmd.k.tff=0;

###################

leg3_cmd.a.p_des = 0.2;
leg3_cmd.a.v_des = 0;
leg3_cmd.a.tff=0;

leg3_cmd.h.p_des = 0.2;
leg3_cmd.h.v_des = 0;
leg3_cmd.h.tff=0;

leg3_cmd.k.p_des = 0.2;
leg3_cmd.k.v_des = 0;
leg3_cmd.k.tff=0;

leg4_cmd.a.p_des = -0.2;
leg4_cmd.a.v_des = 0;
leg4_cmd.a.tff=0;

leg4_cmd.h.p_des = -0.2;
leg4_cmd.h.v_des = 0;
leg4_cmd.h.tff=0;

leg4_cmd.k.p_des = -0.2;
leg4_cmd.k.v_des = 0;
leg4_cmd.k.tff=0;
'''
###################

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000;
checksum_spi0 = 0;
spi0_data=[];

spi1 = spidev.SpiDev()
spi1.open(0, 1)
spi1.max_speed_hz = 1000000;
checksum_spi1 = 0;
spi1_data=[];
def spi0_cmd_pack(enable_flag):
    cmd_pack_list = [leg1_cmd.a.p_des + leg1_state.a.p_offset,leg2_cmd.a.p_des + leg2_state.a.p_offset,
       leg1_cmd.h.p_des + leg1_state.h.p_offset,leg2_cmd.h.p_des + leg2_state.h.p_offset,
       (leg1_cmd.k.p_des + leg1_state.k.p_offset)*leg1_cmd.k.GR,
        (leg2_cmd.k.p_des + leg2_state.k.p_offset)*leg2_cmd.k.GR,
       
       leg1_cmd.a.v_des,leg2_cmd.a.v_des,
       leg1_cmd.h.v_des,leg2_cmd.h.v_des,
       leg1_cmd.k.v_des*leg1_cmd.k.GR,leg2_cmd.k.v_des*leg2_cmd.k.GR,
       
       leg1_cmd.kp_abad,leg2_cmd.kp_abad,
       leg1_cmd.kp_hip, leg2_cmd.kp_hip,
       leg1_cmd.kp_knee,leg2_cmd.kp_knee,
       
       leg1_cmd.kd_abad,leg2_cmd.kd_abad,
       leg1_cmd.kd_hip, leg2_cmd.kd_hip,
       leg1_cmd.kd_knee,leg2_cmd.kd_knee,
       
       leg1_cmd.a.tff,leg2_cmd.a.tff,
       leg1_cmd.h.tff,leg2_cmd.h.tff,
       leg1_cmd.k.tff/leg1_cmd.k.GR,leg2_cmd.k.tff/leg2_cmd.k.GR]
    
    spi0_cmd_packed= encode_spicmd(cmd_pack_list)
    _flag1_enc = (enable_flag&0xffffffff).to_bytes(4,'little')
    _flag2_enc = (enable_flag&0xffffffff).to_bytes(4,'little')
    spi0_cmd_packed.extend(_flag1_enc)
    spi0_cmd_packed.extend(_flag2_enc)
    
    checksum_spi0 = checksum_spi(spi0_cmd_packed)
    checksum_spi0_enc = (checksum_spi0&0xffffffff).to_bytes(4,'big')
    #print(checksum_spi0_enc)
    spi0_cmd_packed.extend(checksum_spi0_enc)
    
    return spi0_cmd_packed
    
def spi1_cmd_pack(enable_flag):
    cmd_pack_list = [leg3_cmd.a.p_des + leg3_state.a.p_offset,leg4_cmd.a.p_des + leg4_state.a.p_offset,
                     leg3_cmd.h.p_des + leg3_state.h.p_offset,leg4_cmd.h.p_des + leg4_state.h.p_offset,
                     (leg3_cmd.k.p_des + leg3_state.k.p_offset)*leg3_cmd.k.GR,
                     (leg4_cmd.k.p_des + leg4_state.k.p_offset)*leg4_cmd.k.GR,
       
       leg3_cmd.a.v_des,leg4_cmd.a.v_des,
       leg3_cmd.h.v_des,leg4_cmd.h.v_des,
       leg3_cmd.k.v_des*leg3_cmd.k.GR,leg4_cmd.k.v_des*leg4_cmd.k.GR,
       
       leg3_cmd.kp_abad,leg4_cmd.kp_abad,
       leg3_cmd.kp_hip, leg4_cmd.kp_hip,
       leg3_cmd.kp_knee,leg4_cmd.kp_knee,
       
       leg3_cmd.kd_abad,leg4_cmd.kd_abad,
       leg3_cmd.kd_hip, leg4_cmd.kd_hip,
       leg3_cmd.kd_knee,leg4_cmd.kd_knee,
       
       leg3_cmd.a.tff,leg4_cmd.a.tff,
       leg3_cmd.h.tff,leg4_cmd.h.tff,
       leg3_cmd.k.tff/leg3_cmd.k.GR,leg4_cmd.k.tff/leg4_cmd.k.GR]
    spi1_cmd_packed= encode_spicmd(cmd_pack_list)
    _flag3_enc = (enable_flag&0xffffffff).to_bytes(4,'little')
    _flag4_enc = (enable_flag&0xffffffff).to_bytes(4,'little')
    spi1_cmd_packed.extend(_flag3_enc)
    spi1_cmd_packed.extend(_flag4_enc)
    checksum_spi1 = checksum_spi(spi1_cmd_packed)
    checksum_spi1_enc = (checksum_spi1&0xffffffff).to_bytes(4,'little')
    #print(checksum_spi1_enc)
    spi1_cmd_packed.extend(checksum_spi1_enc)
    
    return spi1_cmd_packed

def uart_get_data():
    global uart 
    decode_uart_ = []
    try:
        if ser.in_waiting > 0:
            uart = ser.readline();
        if(uart[0] == None or uart[0] ==0xaa):
            decode_uart_ = decode_spidata(uart[1:(29-12)]);
        else:
            decode_uart_ = [0.0, 0.0, 0.0, 0.0]
        decode_uart_.extend(uart[29:32])
        return decode_uart_;
    except:
        ser.close()
        print('uart error')
        return 'uart error'
            
        
def update_leg_data(LF_float_data, LH_float_data):
    leg1_state.a.p = LF_float_data[0] - leg1_state.a.p_offset;
    leg2_state.a.p = LF_float_data[1] - leg2_state.a.p_offset;
    leg3_state.a.p = LH_float_data[0] - leg3_state.a.p_offset;
    leg4_state.a.p = LH_float_data[1] - leg4_state.a.p_offset;
    
    leg1_state.h.p = LF_float_data[2] - leg1_state.h.p_offset;
    leg2_state.h.p = LF_float_data[3] - leg2_state.h.p_offset;
    leg3_state.h.p = LH_float_data[2] - leg3_state.h.p_offset;
    leg4_state.h.p = LH_float_data[1] - leg4_state.h.p_offset;
    
    leg1_state.k.p = LF_float_data[4]/leg1_state.k.GR - leg1_state.k.p_offset;
    leg2_state.k.p = LF_float_data[5]/leg2_state.k.GR - leg2_state.k.p_offset;
    leg3_state.k.p = LH_float_data[4]/leg3_state.k.GR - leg3_state.k.p_offset;
    leg4_state.k.p = LH_float_data[5]/leg4_state.k.GR - leg4_state.k.p_offset;
    
    leg1_state.a.v = LF_float_data[6];
    leg2_state.a.v = LF_float_data[7];
    leg3_state.a.v = LH_float_data[6];
    leg4_state.a.v = LH_float_data[7];
    
    leg1_state.h.v = LF_float_data[8];
    leg2_state.h.v = LF_float_data[9];
    leg3_state.h.v = LH_float_data[8];
    leg4_state.h.v = LH_float_data[9];
    
    leg1_state.k.v = LF_float_data[10]/leg1_state.k.GR;
    leg2_state.k.v = LF_float_data[11]/leg1_state.k.GR;
    leg3_state.k.v = LH_float_data[10]/leg1_state.k.GR;
    leg4_state.k.v = LH_float_data[11]/leg1_state.k.GR;
    
def update_legcmd_plast():
    leg1_cmd.a.p_last = leg1_cmd.a.p_des;
    leg2_cmd.a.p_last = leg2_cmd.a.p_des;
    leg3_cmd.a.p_last = leg3_cmd.a.p_des;
    leg4_cmd.a.p_last = leg4_cmd.a.p_des;
    
    leg1_cmd.h.p_last = leg1_cmd.h.p_des;
    leg2_cmd.h.p_last = leg2_cmd.h.p_des;
    leg3_cmd.h.p_last = leg3_cmd.h.p_des;
    leg4_cmd.h.p_last = leg4_cmd.h.p_des;
    
    leg1_cmd.k.p_last = leg1_cmd.k.p_des;
    leg2_cmd.k.p_last = leg2_cmd.k.p_des;
    leg3_cmd.k.p_last = leg3_cmd.k.p_des;
    leg4_cmd.k.p_last = leg4_cmd.k.p_des;
    

def operation(operation_flag):
    while True:
        if(operation_flag==0):
            pass
        else:
            if(TX_cmd.gait_mode=='stance'):
                pass
            elif(TX_cmd.gait_mode=='crawl'):
                pass
            elif(TX_cmd.gait_mode=='trot'):
                pass
            

for i in range(20):

    spi0_tx = spi0_cmd_pack(0x00)
    #bbb = decode_spidata(spi0_tx)
    spi0_rx = spi.xfer3(spi0_tx)
    spi0_data = decode_spidata(spi0_rx[1:61-12])
    spi0_flag1 = spi0_rx[61-12:61-8]
    spi0_flag2 = spi0_rx[61-8:61-4]
    spi0_checksum = spi0_rx[61-4:61]


    spi1_tx = spi1_cmd_pack(0x00)
    #ccc = decode_spidata(spi1_tx)
    spi1_rx = spi1.xfer3(spi1_tx)
    spi1_data = decode_spidata(spi1_rx[1:61-12])
    spi1_flag3 = spi1_rx[61-12:61-8]
    spi1_flag4 = spi1_rx[61-8:61-4]
    spi1_checksum = spi1_rx[61-4:61]
    RC_data = decode_spidata(spi1_rx[62:90])
    
    leg1_state.a.p_offset = spi0_data[0] - leg1_state.a.p_init;
    leg2_state.a.p_offset = spi0_data[1] - leg2_state.a.p_init;
    leg3_state.a.p_offset = spi1_data[0] - leg3_state.a.p_init;
    leg4_state.a.p_offset = spi1_data[1] - leg4_state.a.p_init;
                    
    leg1_state.h.p_offset = spi0_data[2] - leg1_state.h.p_init;
    leg2_state.h.p_offset = spi0_data[3] - leg2_state.h.p_init;
    leg3_state.h.p_offset = spi1_data[2] - leg3_state.h.p_init;
    leg4_state.h.p_offset = spi1_data[3] - leg4_state.h.p_init;
                    
    leg1_state.k.p_offset = spi0_data[4]/leg1_state.k.GR - leg1_state.k.p_init;
    leg2_state.k.p_offset = spi0_data[5]/leg2_state.k.GR - leg2_state.k.p_init;
    leg3_state.k.p_offset = spi1_data[4]/leg3_state.k.GR - leg3_state.k.p_init;
    leg4_state.k.p_offset = spi1_data[5]/leg4_state.k.GR - leg4_state.k.p_init;
                    
cur_time =0.0

def spi_send_receive(_T):
    global RC_data, spi1_checksum, spi0_checksum, spi0_tx, spi0_rx, spi0_data, spi1_tx, spi1_rx, spi1_data,decode_uart, cur_time;
    _last_time = time.time()
    _dt =0;
    _dt_1khz_cnt = 0;
    _joint_trjactory_Cycle = 0.025;
    a___ = np.hstack((np.array([[0,117.25,-185]]).T,np.array([[0,-117.25,-185]]).T,
                      np.array([[0,117.25,-185]]).T,np.array([[0,-117.25,-185]]).T));
    while True:
        try:
            _dt = time.time() - _last_time;
            
            if(_dt<_T):
                continue;
            
            else:
                _dt_1khz_cnt+=1;
                #print(_dt_1khz_cnt%100, '100hz');
                '''
                if(_dt_1khz_cnt%100==0):
                    #print(_dt_1khz_cnt%100, '100hz');
                    RC_data = uart_get_data();
                    TX_cmd.get_cmd_data(RC_data);
                    TX_cmd.print_cmd_data();
                    '''
                
                #print(_dt_1khz_cnt);
                RC_data = uart_get_data();
                
                TX_cmd.get_cmd_data(RC_data);
                TX_cmd.print_cmd_data();
                
                #leg1_cmd.a.p_des+=0.1
                #leg2_cmd.a.p_des+=0.1
                #leg3_cmd.a.p_des=-0.1
                #leg4_cmd.a.p_des=0.6
                #leg4_cmd.h.p_des=0

                spi0_tx = spi0_cmd_pack(TX_cmd.enable)
                #bbb = decode_spidata(spi0_tx)
                spi0_rx = spi.xfer3(spi0_tx)
                spi0_data = decode_spidata(spi0_rx[1:61-12])
                spi0_flag1 = spi0_rx[61-12:61-8]
                spi0_flag2 = spi0_rx[61-8:61-4]
                spi0_checksum = spi0_rx[61-4:61]
                
                
                spi1_tx = spi1_cmd_pack(TX_cmd.enable)
                #ccc = decode_spidata(spi1_tx)
                spi1_rx = spi1.xfer3(spi1_tx)
                spi1_data = decode_spidata(spi1_rx[1:61-12])
                spi1_flag3 = spi1_rx[61-12:61-8]
                spi1_flag4 = spi1_rx[61-8:61-4]
                spi1_checksum = spi1_rx[61-4:61]
                #RC_data = decode_spidata(spi1_rx[62:90])
                #TX_cmd.get_cmd_data(RC_data)
                #print(RC_data);
                #TX_cmd.print_cmd_data();
                #print(_dt,_T,1/_dt,1/_T)
                #print(leg3_cmd.h.p_des ,'  ', leg3_state.h.p_offset,'  ',leg4_cmd.h.p_des ,'  ',leg4_state.h.p_offset)
                #print(spi0_data[0:6])
                #print(leg2_state.k.p,'\t', leg2_state.k.p_init,'\t', leg2_state.k.p_offset,'\t', spi0_data[5])
                #print(spi1_data[0:6])
                
                #time.sleep(0.005);
                update_leg_data(spi0_data, spi1_data);
                if(TX_cmd.enable==0):
                    '''
                    leg1_state.a.p_offset = spi0_data[0] - leg1_state.a.p_init;
                    leg2_state.a.p_offset = spi0_data[1] - leg2_state.a.p_init;
                    leg3_state.a.p_offset = spi1_data[0] - leg3_state.a.p_init;
                    leg4_state.a.p_offset = spi1_data[1] - leg4_state.a.p_init;
                    
                    leg1_state.h.p_offset = spi0_data[2] - leg1_state.h.p_init;
                    leg2_state.h.p_offset = spi0_data[3] - leg2_state.h.p_init;
                    leg3_state.h.p_offset = spi1_data[2] - leg3_state.h.p_init;
                    leg4_state.h.p_offset = spi1_data[3] - leg4_state.h.p_init;
                    
                    leg1_state.k.p_offset = spi0_data[4]/leg1_state.k.GR - leg1_state.k.p_init;
                    leg2_state.k.p_offset = spi0_data[5]/leg1_state.k.GR - leg2_state.k.p_init;
                    leg3_state.k.p_offset = spi1_data[4]/leg1_state.k.GR - leg3_state.k.p_init;
                    leg4_state.k.p_offset = spi1_data[5]/leg1_state.k.GR - leg4_state.k.p_init;
                    '''
                    
                    leg1_cmd.a.p_des = leg1_state.a.p_init;
                    leg2_cmd.a.p_des = leg2_state.a.p_init;
                    leg3_cmd.a.p_des = leg3_state.a.p_init;
                    leg4_cmd.a.p_des = leg4_state.a.p_init;
                    
                    leg1_cmd.h.p_des = leg1_state.h.p_init;
                    leg2_cmd.h.p_des = leg2_state.h.p_init;
                    leg3_cmd.h.p_des = leg3_state.h.p_init;
                    leg4_cmd.h.p_des = leg4_state.h.p_init;
                    
                    leg1_cmd.k.p_des = leg1_state.k.p_init;
                    leg2_cmd.k.p_des = leg2_state.k.p_init;
                    leg3_cmd.k.p_des = leg3_state.k.p_init;
                    leg4_cmd.k.p_des = leg4_state.k.p_init;
                    
                    #print(spi0_data[5], '\t', (leg2_cmd.k.p_des + leg2_state.k.p_offset)*leg2_cmd.k.GR)
                elif (TX_cmd.enable==1):
                    pass
                    #print(spi0_data[5], '\t', (leg2_cmd.k.p_des + leg2_state.k.p_offset)*leg2_cmd.k.GR)
                elif (TX_cmd.enable == 0xf and TX_cmd.standby == 0):
                    update_legcmd_plast();
                    
                    leg4_cmd.k.p_des = 90*_PI_/180;
                    leg3_cmd.k.p_des = -90*_PI_/180;
                    leg1_cmd.k.p_des = -90*_PI_/180;
                    leg2_cmd.k.p_des = 90*_PI_/180;
                    
                    leg1_cmd.h.p_des = -135*_PI_/180;
                    leg3_cmd.h.p_des = -135*_PI_/180;
                    leg2_cmd.h.p_des =  135*_PI_/180;
                    leg4_cmd.h.p_des =  135*_PI_/180;
                    #a___ = np.hstack(([0,117.25,-185],[0,-117.25,-185],[0,117.25,-185],[0,-117.25,-185]));
                    print('a___',a___);
                    cur_time = time.time();
                elif (TX_cmd.enable == 0xf and TX_cmd.standby == 1):
                    if(_dt_1khz_cnt%5==0):
                        update_legcmd_plast();
                    a___ = standing_pose(185,TX_cmd.dz,TX_cmd.droll,TX_cmd.dpitch,TX_cmd.dyaw)
                        
                    #print(a___[0:3,0])
                    _t11_t12_t13 = inverse_kinematic1(a___[0:3,0]);
                    _t21_t22_t23 = inverse_kinematic2(a___[0:3,1]);
                    qqqqq = convert_angle3(_t21_t22_t23);
                    print('cvt',qqqqq);
                    #q = create_joint_velocity_profile(_dt_1khz_cnt*_T, _joint_trjactory_Cycle,leg1_state.k.p,convert_angle(_t11_t12_t13[2]))
                    q2 = create_joint_velocity_profile2(_dt_1khz_cnt*0.001, _joint_trjactory_Cycle,leg2_cmd.k.p_last ,convert_angle(_t21_t22_t23[2]))
                    q22 = create_joint_velocity_profile2(_dt_1khz_cnt*0.001, _joint_trjactory_Cycle,leg2_cmd.h.p_last ,convert_angle(_t21_t22_t23[1]))
                    q2__ = create_joint_velocity_profile__(_dt_1khz_cnt*_T, _joint_trjactory_Cycle,[leg2_state.a.p,leg2_state.h.p,leg2_state.k.p],qqqqq) 
                    leg1_cmd.a.p_des = convert_angle(_t11_t12_t13[0]);
                    leg1_cmd.h.p_des = convert_angle(_t11_t12_t13[1]);
                    leg1_cmd.k.p_des = convert_angle(_t11_t12_t13[2]);
                    
                    leg2_cmd.a.p_des = convert_angle(_t21_t22_t23[0]);
                    #leg2_cmd.h.p_des = convert_angle(_t21_t22_t23[1]);
                    #leg2_cmd.k.p_des = convert_angle(_t21_t22_t23[2]);
                    
                    #leg2_cmd.a.p_des = q2__[0][0];
                    #leg2_cmd.h.p_des = q2__[0][1];
                    #leg2_cmd.k.p_des = q2__[0][2];
                    leg2_cmd.h.p_des = q22[0];
                    leg2_cmd.k.p_des = q2[0];
                    
                    #print('traj',qqqqq, '\t', q2__[0]);
                    #print('legstate  ',leg2_state.k.p,'\t',convert_angle(_t21_t22_t23[2]),'\t', q2);
                    print('p_des_p_last ',leg2_cmd.k.p_last, leg2_cmd.k.p_des);
                    #print('t11:',leg1_cmd.a.p_des,
                    #  't12:',leg1_cmd.h.p_des,
                    #  't13:',leg1_cmd.k.p_des)
                    
                    #a___ = standing_pose(185,TX_cmd.dz,TX_cmd.droll,TX_cmd.dpitch,TX_cmd.dyaw)
                    #print(a___)
                                       
                    

                
                _last_time = time.time()
                
        except:
            print("spi_err");
            spi.close()
            spi1.close()
            ser.close()
            return 0
 
def start_up_robot():
    global spi1_data, spi0_data;
    try:
        while True:
            if(TX_cmd.enable==0):
                leg1_state.a.p_offset = spi0_data[0] - leg1_state.a.p_init;
                leg2_state.a.p_offset = spi0_data[1] - leg2_state.a.p_init;
                leg3_state.a.p_offset = spi1_data[0] - leg3_state.a.p_init;
                leg4_state.a.p_offset = spi1_data[1] - leg4_state.a.p_init;
                
                leg1_state.h.p_offset = spi0_data[2] - leg1_state.h.p_init;
                leg2_state.h.p_offset = spi0_data[3] - leg2_state.h.p_init;
                leg3_state.h.p_offset = spi1_data[2] - leg3_state.h.p_init;
                leg4_state.h.p_offset = spi1_data[3] - leg4_state.h.p_init;
                
                leg1_state.k.p_offset = spi0_data[4]/leg1_state.k.GR - leg1_state.k.p_init;
                leg2_state.k.p_offset = spi0_data[5]/leg1_state.k.GR - leg2_state.k.p_init;
                leg3_state.k.p_offset = spi1_data[4]/leg1_state.k.GR - leg3_state.k.p_init;
                leg4_state.k.p_offset = spi1_data[5]/leg1_state.k.GR - leg4_state.k.p_init;
                
                
                leg1_cmd.a.p_des = leg1_state.a.p_init;
                leg2_cmd.a.p_des = leg2_state.a.p_init;
                leg3_cmd.a.p_des = leg3_state.a.p_init;
                leg4_cmd.a.p_des = leg4_state.a.p_init;
                
                leg1_cmd.h.p_des = leg1_state.h.p_init;
                leg2_cmd.h.p_des = leg2_state.h.p_init;
                leg3_cmd.h.p_des = leg3_state.h.p_init;
                leg4_cmd.h.p_des = leg4_state.h.p_init;
                
                leg1_cmd.k.p_des = leg1_state.k.p_init;
                leg2_cmd.k.p_des = leg2_state.k.p_init;
                leg3_cmd.k.p_des = leg3_state.k.p_init;
                leg4_cmd.k.p_des = leg4_state.k.p_init;
            elif (TX_cmd.enable==1):
                pass
                
            elif (TX_cmd.enable == 0xf and TX_cmd.standby == 0):
                #leg1_cmd.enable=1;
                #leg2_cmd.enable=1;
                #leg3_cmd.enable=1;
                #leg4_cmd.enable=1;
                leg4_cmd.k.p_des = 90*math.pi/180;
                leg3_cmd.k.p_des = -90*math.pi/180;
                leg1_cmd.k.p_des = -90*math.pi/180;
                leg2_cmd.k.p_des = 90*math.pi/180;
                
                leg1_cmd.h.p_des = -135*math.pi/180;
                leg3_cmd.h.p_des = -135*math.pi/180;
                leg2_cmd.h.p_des =  135*math.pi/180;
                leg4_cmd.h.p_des =  135*math.pi/180;
            elif (TX_cmd.enable == 0xf and TX_cmd.standby == 1):
                _t11_t12_t13 = inverse_kinematic1([0, 117.25,-185]);
                _t21_t22_t23 = inverse_kinematic2([0,-117.25,-185]);
                leg1_cmd.a.p_des = convert_angle(_t11_t12_t13[0]);
                leg1_cmd.h.p_des = convert_angle(_t11_t12_t13[1]);
                leg1_cmd.k.p_des = convert_angle(_t11_t12_t13[2]);
                
                leg2_cmd.a.p_des = convert_angle(_t21_t22_t23[0]);
                leg2_cmd.h.p_des = convert_angle(_t21_t22_t23[1]);
                leg2_cmd.k.p_des = convert_angle(_t21_t22_t23[2]);
                '''              
                print('t11:',convert_angle(_t11_t12_t13[0]),
                      't12:',convert_angle(_t11_t12_t13[1]),
                      't13:',convert_angle(_t11_t12_t13[2]))
                print('t21:',convert_angle(_t21_t22_t23[0]),
                      't22:',convert_angle(_t21_t22_t23[1]),
                      't23:',convert_angle(_t21_t22_t23[2]))
                '''
            
            
    except:
        print('start up fail')
        return 0
                    
#trot_planer(start_time, T_gait_cycle,dx, dy, dz, z, yaw, gaitmode):


thread1 = threading.Thread(target = spi_send_receive, args=(0.001,))
#thread2 = threading.Thread(target = start_up_robot)
thread1.start()
#thread2.start()
'''
try:
    
    pass
        
except KeyboardInterrupt:
    spi.close()
'''
        



