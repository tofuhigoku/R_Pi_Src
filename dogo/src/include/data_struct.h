#ifndef data_struct_H
#define data_struct_H

#include "stdio.h"
#include "stdlib.h"
#include <stdint.h>
#include "string.h"
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>


typedef struct	// size = 12 bytes
{
	float p;
	float v;
	float t;
} joint_data_t;

typedef struct	// size = 12*3 = 36 bytes
{
	joint_data_t abad;
	joint_data_t hip;
	joint_data_t knee;
	
} leg_data_t;


typedef struct // size = 20 bytes
{
    float p_cmd;
		float v_cmd;
		float	kp;
		float kd;
		float torque_cmd;
	
}joint_control_t;
   
typedef struct  // size = 3*20 = 60 bytes
{
	
	joint_control_t abad;
	joint_control_t hip;
	joint_control_t knee;
	
}leg_control_t;




typedef struct{
	uint16_t CH_[7];    // raw data from mcu
	// uint8_t XYZyaw_gait_sp_st[7];
	// float  XYZyaw_gait_sp_st_[7];
	// uint16_t X, Y, Z, yaw; //ch2, ch1, ch3, ch4
	// uint16_t gait, speed, standby; //ch5, ch6, ch7

}RX_signal_decode;



typedef struct				// size = 84 bytes
{
//    float q_abad[2];
//    float q_hip[2];
//    float q_knee[2];
//    float qd_abad[2];
//    float qd_hip[2];
//    float qd_knee[2];
	
		leg_data_t leg1_data;	// 36 bytes
		leg_data_t leg2_data;	// 36 bytes
	
    int32_t flags[2];	// 8 bytes
    int32_t checksum;	// 4 bytes
	
} spi_data_t ;


typedef struct	// size = 132 bytes
{
//    float q_des_abad[2];
//    float q_des_hip[2];
//    float q_des_knee[2];
//    float qd_des_abad[2];
//    float qd_des_hip[2];
//    float qd_des_knee[2];
//    float kp_abad[2];
//    float kp_hip[2];
//    float kp_knee[2];
//    float kd_abad[2];
//    float kd_hip[2];
//    float kd_knee[2];
//    float tau_abad_ff[2];
//    float tau_hip_ff[2];
//    float tau_knee_ff[2];
	
		leg_control_t leg1_cmd;		// 60 bytes
		leg_control_t leg2_cmd;		// 60 bytes
	
    int32_t flags[2];					// 8 bytes
    uint32_t checksum;				// 4 bytes
	
} spi_command_t;


#endif