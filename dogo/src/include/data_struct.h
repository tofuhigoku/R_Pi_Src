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



#endif