#define _USE_MATH_DEFINES
#include "data_struct.h"
#include "QuadrupedRobotObject.h"


#include <glm/glm.hpp>
#include <glm/trigonometric.hpp>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

QuadrupedRobotObject OwO;

float T_gaitcycle = 0.5;       // 0.5s
float simulation_period = 2;   // 2s
float time_step = 0.001;


int vector_size = (int) simulation_period/time_step;
vector<float>  x1_vector(vector_size);
vector<float>  t_vector(vector_size);
vector<float>  y1_vector(vector_size);
vector<float>  z1_vector(vector_size);

float default_x = 0, default_y = 60, default_z = -180;

void Troting_path(float x_step, float y_step, float z_step, float time, int i )
{
    float _dt = fmodf(time, T_gaitcycle);
    cout << "time: " << time << " dt: " << _dt << endl;
    float x = 0, y = 0, z = 0;
    float z_phase = 0;
    if(_dt > T_gaitcycle*0.5)
    {
        z_phase = 1;
    }
    
    x = (x_step*(glm::sin(_dt/(0.25*T_gaitcycle)*_PI_2+_3PI_2)*0.5 - 0.5) + default_x);
    y = (y_step*(glm::sin(_dt/(0.25*T_gaitcycle)*_PI_2+_3PI_2)*0.5 - 0.5) + default_y);
    z = (z_step*(glm::sin(_dt/(0.50*T_gaitcycle)*_PI))*(1 - z_phase) + default_z);

// self.desired_pos[0,0] = ((self.d_xyzyaw[0])*(math.sin((t/(0.25*self._pattern_planner.trot_T ))*pi*0.5+3*pi/2)*0.5-0.5) + __tmp_arr[0,0])
// self.desired_pos[1,0] = ((self.d_xyzyaw[1])*(math.sin((t/(0.25*self._pattern_planner.trot_T ))*pi*0.5+3*pi/2)*0.5-0.5) + __tmp_arr[1,0])
// self.desired_pos[2,0] = ((self.d_xyzyaw[2])*(math.sin((t/(0.5*self._pattern_planner.trot_T ))*pi))*(1-self._pattern_planner.current_phase[0]) + __tmp_arr[2,0])

    x1_vector[i] = x;
    y1_vector[i] = y;
    z1_vector[i] = z;
    t_vector[i] = time;
}


int main()
{
    OwO.foot_step = glm::vec3{50, 0 , 50};
    float time = 0 ;
    int i = 0;
    while(time<= simulation_period)
    {
        Troting_path(50, 0, 80, time, i );
        time +=time_step;
        i++;
    }
    glm::vec3 U(1,2,3);
    std::vector<double> A(3);
    for (int i =0; i < A.size(); i++)
    {
        A[i] = i;
    }
    plt::subplot(3, 1, 1); // 2 rows, 1 column, first plot
    plt::plot(t_vector, x1_vector);
    plt::subplot(3, 1, 2); // 2 rows, 1 column, second plot
    plt::plot(t_vector, y1_vector);
    plt::subplot(3, 1, 3); // 2 rows, 1 column, second plot
    plt::plot(t_vector, z1_vector);
    plt::show();
}

// #include "matplotlibcpp.h"
// #include <vector>
// #include <cmath>

// namespace plt = matplotlibcpp;

// int main() {
//     std::vector<double> t(1000);
//     std::vector<double> x(t.size());

//     for(size_t i = 0; i < t.size(); i++) {
//         t[i] = i / 100.0;
//         x[i] = sin(2.0 * M_PI * 1.0 * t[i]);
//     }

//     plt::xkcd();
//     plt::plot(t, x);
//     plt::title("AN ORDINARY SIN WAVE");
//     plt::save("xkcd.png");
// }