#ifndef QuadrupedRobotObject_H
#define QuadrupedRobotObject_H

#include "stdio.h"
#include "stdlib.h"
#include <stdint.h>
#include "string.h"
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <eigen3/Eigen/Dense>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "api.h"
#include "data_struct.h"

#define DISABLE_CMD			0X00
#define ENABLE_CMD			0XEC
#define PHASE_ALIGN_CMD	    0XAC

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

class QuadrupedRobotObject
{
private:
    /* data */
public:
    /* data */
    spi_command_t   spi_device0_cmd;
    spi_command_t   spi_device1_cmd;

    spi_data_t      spi_device0_data;
    spi_data_t      spi_device1_data;

    



    /* method */
    QuadrupedRobotObject(/* args */);
    ~QuadrupedRobotObject();
};

QuadrupedRobotObject::QuadrupedRobotObject(/* args */)
{
}

QuadrupedRobotObject::~QuadrupedRobotObject()
{
}


#endif