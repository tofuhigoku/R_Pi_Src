#include "data_struct.h"
#include "QuadrupedRobotObject.h"


#include <glm/glm.hpp>
#include <glm/trigonometric.hpp>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

QuadrupedRobotObject OwO;

float T_gaitcycle = 0.5;       // 0.5s
float simulation_period = 2;   // 2s

glm::vec3  Link1_hip;
// glm::

void Troting_path(float x_step, float y_step, float z_step, float time )
{
    float _dt = fmodf(time, T_gaitcycle);
    cout << "time: " << time << " dt: " << _dt << endl;

}


int main()
{
    OwO.foot_step = glm::vec3{50, 0 , 50};
    float time = 0 ;
    while(time<= simulation_period)
    {
        Troting_path(80, 0, 80, time);
        time +=0.01;
    }
    glm::vec3 U(1,2,3);
    std::vector<double> A(3);
    for (int i =0; i < A.size(); i++)
    {
        A[i] = i;
    }
    plt::plot(A,A);
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