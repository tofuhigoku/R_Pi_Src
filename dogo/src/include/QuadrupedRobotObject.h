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
#include <glm/trigonometric.hpp>

#include "api.h"
#include "data_struct.h"

#define DISABLE_CMD			0X00
#define ENABLE_CMD			0XEC
#define PHASE_ALIGN_CMD	    0XAC

#define _PI                 3.141592653589793
#define _PI_2               3.141592653589793/2
#define _PI_3               3.141592653589793/3

#define _2PI                3.141592653589793*2
#define _3PI_2              3*3.141592653589793/2

using namespace std;

typedef struct
{
	/* data */
	leg_data_t	current_state;

	/* command */
	leg_control_t desired_state;

	
} Leg_t;
typedef struct
{

	Leg_t	leg1;
	Leg_t	leg2;
	Leg_t	leg3;
	Leg_t	leg4;
	
} RobotLeg_t;

typedef struct
{
    float L, W, OA, AB, BC;
} RobotDimension_t;

typedef struct
{
    float Ixx, Iyy, Izz;
} Inertial_by_Axis_t;

typedef struct
{
    float Body_Weight;

    Inertial_by_Axis_t  body;

    Inertial_by_Axis_t  Link11;
    Inertial_by_Axis_t  Link12;
    Inertial_by_Axis_t  Link13;

    Inertial_by_Axis_t  Link21;
    Inertial_by_Axis_t  Link22;
    Inertial_by_Axis_t  Link23;

    Inertial_by_Axis_t  Link31;
    Inertial_by_Axis_t  Link32;
    Inertial_by_Axis_t  Link33;

    Inertial_by_Axis_t  Link41;
    Inertial_by_Axis_t  Link42;
    Inertial_by_Axis_t  Link43;

} RobotInertial_param_t;

typedef struct
{
    glm::mat3x3  body;

    glm::mat3x3  Link1_hip;
    glm::mat3x3  Link1_Uleg;
    glm::mat3x3  Link1_Lleg;

    glm::mat3x3  Link2_hip;
    glm::mat3x3  Link2_Uleg;
    glm::mat3x3  Link2_Lleg;

    glm::mat3x3  Link3_hip;
    glm::mat3x3  Link3_Uleg;
    glm::mat3x3  Link3_Lleg;

    glm::mat3x3  Link4_hip;
    glm::mat3x3  Link4_Uleg;
    glm::mat3x3  Link4_Lleg;

} RobotInertial_matrix_t;

typedef struct
{
    glm::vec3  body;

    glm::vec3  Link1_hip;
    glm::vec3  Link1_Uleg;
    glm::vec3  Link1_Lleg;

    glm::vec3  Link2_hip;
    glm::vec3  Link2_Uleg;
    glm::vec3  Link2_Lleg;

    glm::vec3  Link3_hip;
    glm::vec3  Link3_Uleg;
    glm::vec3  Link3_Lleg;

    glm::vec3  Link4_hip;
    glm::vec3  Link4_Uleg;
    glm::vec3  Link4_Lleg;

} RobotCGpos_matrix_t;


class QuadrupedRobotObject
{
private:
    /* data */
    RobotDimension_t RobotDimension;
    float weight;
    RobotInertial_param_t   RobotInertial_param;
    RobotInertial_matrix_t  RobotInertial_matrix;

    glm::mat4 T_h1_11 = glm::mat4(1.0);
    glm::mat4 T_h2_21 = glm::mat4(1.0);
    glm::mat4 T_h3_31 = glm::mat4(1.0);
    glm::mat4 T_h4_41 = glm::mat4(1.0);

    float T_cycle;
    float t_cycle;
    /* private methods */
    float update_T_cycle(float _T_cycle);
    float get_T_cycle();
    float update_t_cycle(float _t_cycle);
    float get_t_cycle();

    int R_RobotInertial_Init();
    
    int R_Leg_FK_Lside( glm::vec3 Vec3_LegJoints_Pos, glm::vec3 & Vec3_Foot_Pos);
    int R_Leg_FK_Rside( glm::vec3 Vec3_LegJoints_Pos, glm::vec3 & Vec3_Foot_Pos);


    int R_Leg_IK_Lside( glm::vec3 & Vec3_LegJoints_Pos, glm::vec3 Vec3_Foot_Pos);
    int R_Leg_IK_Rside( glm::vec3 & Vec3_LegJoints_Pos, glm::vec3 Vec3_Foot_Pos);


public:
    /* data */

    spi_command_t   spi_device0_cmd;
    spi_command_t   spi_device1_cmd;

    spi_data_t      spi_device0_data;
    spi_data_t      spi_device1_data;

    RobotLeg_t RobotLeg;
    glm::vec3 foot_step;
    /* method */
    QuadrupedRobotObject(/* args */);
    ~QuadrupedRobotObject();

    void generate_Body_state();
    void generate_Crawling_gait();
    void generate_Trotting_gait();

    int R_All_Leg_FK();
    int R_All_Leg_IK();

    int R_Laydown();
};

QuadrupedRobotObject::QuadrupedRobotObject(/* args */)
{
}

QuadrupedRobotObject::~QuadrupedRobotObject()
{
}

float QuadrupedRobotObject::update_T_cycle(float _T_cycle)
{
    T_cycle = _T_cycle;
    return T_cycle;
}
float QuadrupedRobotObject::get_T_cycle()
{
    return T_cycle;
}
float QuadrupedRobotObject::update_t_cycle(float _t_cycle)
{
    t_cycle = _t_cycle;
    return t_cycle;
}
float QuadrupedRobotObject::get_t_cycle()
{
    return t_cycle;
}


int QuadrupedRobotObject::R_Leg_FK_Lside( glm::vec3 Vec3_LegJoints_Pos, glm::vec3 & Vec3_Foot_Pos)
{
    float theta1 = Vec3_LegJoints_Pos[0];
    float theta2 = Vec3_LegJoints_Pos[1];
    float theta3 = Vec3_LegJoints_Pos[2];

    float a00 =     glm::cos(theta1)*glm::cos(theta2+theta3);
    float a01 = -1* glm::cos(theta1)*glm::sin(theta2+theta3);
    float a02 = -1* glm::sin(theta1);
    float a03 =     RobotDimension.AB* glm::cos(theta1)*glm::cos(theta2) - RobotDimension.OA*glm::sin(theta1) + RobotDimension.BC*glm::cos(theta1)*glm::cos(theta2+theta3);

    float a10 =     glm::sin(theta1)*glm::cos(theta2+theta3);
    float a11 = -1* glm::sin(theta1)*glm::sin(theta2+theta3);
    float a12 =     glm::cos(theta1);
    float a13 =     RobotDimension.AB* glm::sin(theta1)*glm::cos(theta2) + RobotDimension.OA*glm::cos(theta1) + RobotDimension.BC*glm::sin(theta1)*glm::cos(theta2+theta3);

    float a20 = -1* glm::sin(theta2+theta3);
    float a21 = -1* glm::cos(theta2+theta3);
    float a22 =     0;
    float a23 = -1* RobotDimension.AB*glm::sin(theta2) - RobotDimension.BC*glm::sin(theta2+theta3);

    glm::mat4x4 T_11_14 = glm::transpose(glm::mat4x4{
        {a00, a01, a02, a03},
        {a10, a11, a12, a13},
        {a20, a21, a22, a23},
        {0  , 0  , 0  , 1  }      
        });
    glm::mat4x4 T_h1_14 = T_h1_11*T_11_14; 

}

int QuadrupedRobotObject::R_Leg_FK_Rside( glm::vec3 Vec3_LegJoints_Pos, glm::vec3 & Vec3_Foot_Pos)
{
    float theta1 = Vec3_LegJoints_Pos[0];
    float theta2 = Vec3_LegJoints_Pos[1];
    float theta3 = Vec3_LegJoints_Pos[2];

    float a00 =     glm::cos(theta1)*glm::cos(theta2+theta3);
    float a01 = -1* glm::cos(theta1)*glm::sin(theta2+theta3);
    float a02 = -1* glm::sin(theta1);
    float a03 =     RobotDimension.AB* glm::cos(theta1)*glm::cos(theta2) - RobotDimension.OA*glm::sin(theta1) + RobotDimension.BC*glm::cos(theta1)*glm::cos(theta2+theta3);

    float a10 =     glm::sin(theta1)*glm::cos(theta2+theta3);
    float a11 = -1* glm::sin(theta1)*glm::sin(theta2+theta3);
    float a12 =     glm::cos(theta1);
    float a13 =     RobotDimension.AB* glm::sin(theta1)*glm::cos(theta2) + RobotDimension.OA*glm::cos(theta1) + RobotDimension.BC*glm::sin(theta1)*glm::cos(theta2+theta3);

    float a20 = -1* glm::sin(theta2+theta3);
    float a21 = -1* glm::cos(theta2+theta3);
    float a22 =     0;
    float a23 = -1* RobotDimension.AB*glm::sin(theta2) - RobotDimension.BC*glm::sin(theta2+theta3);

    glm::mat4x4 T_21_24 = glm::transpose(glm::mat4x4{
        {a00, a01, a02, a03},
        {a10, a11, a12, a13},
        {a20, a21, a22, a23},
        {0  , 0  , 0  , 1  }      
        });
    glm::mat4x4 T_h2_24 = T_h2_21*T_21_24; 

}

int QuadrupedRobotObject::R_Leg_IK_Lside( glm::vec3 & Vec3_LegJoints_Pos, glm::vec3 Vec3_Foot_Pos)
{
    float x = Vec3_Foot_Pos.x;
    float y = Vec3_Foot_Pos.y;
    float z = Vec3_Foot_Pos.z;

    float OD = glm::sqrt(y*y + z*z);
    // float z_new = - glm::sqrt(y*y + z*z - RobotDimension.OA*RobotDimension.OA);
    float AC_square = x*x + (y*y + z*z - RobotDimension.OA*RobotDimension.OA);
    float AB_square = RobotDimension.AB*RobotDimension.AB;
    float BC_square = RobotDimension.BC*RobotDimension.BC;
    float AC = glm::sqrt(AC_square);

    /* Calculate q1*/
    float q1 = 0.0f;
    {
        float alpha = glm::acos(y/OD);
        float beta  = glm::acos(RobotDimension.OA/OD);
        if(y>=0)
        {
            q1 = glm::abs(alpha) - glm::abs(beta);
        }
        else
        {
            q1 = _PI - glm::abs(alpha) - glm::abs(beta);
        }
    }

    /* Calculate q2*/
    float q2 = 0.0f;
    {
        float PHI = glm::acos(x/AC);
        float phi = glm::acos((AB_square + AC_square - BC_square) /(2*RobotDimension.AB*AC));

        if(x<0)
        {
            q2 = -_PI_2 + glm::abs(PHI) - glm::abs(phi);
        }
        else
        {
            q2 = _PI_2 - glm::abs(PHI) - glm::abs(phi);
        }
    }

    /* Calculate q3*/
    float q3 =  glm::abs(
                glm::acos(-(AB_square + BC_square - AC_square) /(2*RobotDimension.AB*RobotDimension.BC)));

    /* assign value to joints*/
    Vec3_LegJoints_Pos[0] = q1;
    Vec3_LegJoints_Pos[1] = _PI - q2;
    Vec3_LegJoints_Pos[2] = _2PI - q3;

}
int QuadrupedRobotObject::R_Leg_IK_Rside( glm::vec3 & Vec3_LegJoints_Pos, glm::vec3 Vec3_Foot_Pos)
{
    float x = Vec3_Foot_Pos.x;
    float y = Vec3_Foot_Pos.y;
    float z = Vec3_Foot_Pos.z;


    float OD = glm::sqrt(y*y + z*z);
    // float z_new = - glm::sqrt(y*y + z*z - RobotDimension.OA*RobotDimension.OA);
    float AC_square = x*x + (y*y + z*z - RobotDimension.OA*RobotDimension.OA);
    float AB_square = RobotDimension.AB*RobotDimension.AB;
    float BC_square = RobotDimension.BC*RobotDimension.BC;
    float AC = glm::sqrt(AC_square);

    /* Calculate q1*/
    float q1 = 0.0f;
    {
        float alpha = glm::acos(y/OD);
        float beta  = glm::acos(RobotDimension.OA/OD);

        if(y<0)
        {
            q1 = glm::abs(alpha) - glm::abs(beta);
        }
        else
        {
            q1 = _PI - glm::abs(alpha) - glm::abs(beta);
        }
    }
    
    /* Calculate q2*/
    float q2 = 0.0f;
    {
        float PHI = glm::acos(x/AC);
        float phi = glm::acos((AB_square + AC_square - BC_square) /(2*RobotDimension.AB*AC));

        if(x<0)
        {
            q2 = -_PI_2 + glm::abs(PHI) - glm::abs(phi);
        }
        else
        {
            q2 = _PI_2 - glm::abs(PHI) - glm::abs(phi);
        }
    }

    /* Calculate q3*/
    float q3 =  glm::abs(
                glm::acos(-(AB_square + BC_square - AC_square) /(2*RobotDimension.AB*RobotDimension.BC)));

    /* assign value to joints*/
    Vec3_LegJoints_Pos[0] = q1;
    Vec3_LegJoints_Pos[1] = _PI + q2;
    Vec3_LegJoints_Pos[2] = q3;
}

void QuadrupedRobotObject::generate_Body_state()
{
    /* In construction*/
}
void QuadrupedRobotObject::generate_Crawling_gait()
{
    /* In construction*/
}
void QuadrupedRobotObject::generate_Trotting_gait()
{
    /* In construction*/
}
#endif