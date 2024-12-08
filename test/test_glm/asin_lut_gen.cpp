#include <iostream>
#include <vector>


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
#include <glm/gtx/io.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

using namespace std;

vector<float> asin_lut;

int main(int argc, char** argv)
{
    // int lut_size = atoi(argv[1]);
    // cout << "lut size = " << lut_size << endl;
    // float step_size = 2.0f / lut_size;
    // cout << "step size = " << step_size << endl;
    // for (int i = 0; i < lut_size; i++)
    // {
    //     float var = -1 + step_size*i;
    //     float a = glm::asin((double) var);
    //     asin_lut.push_back( a);
    //     cout << "glm::acos((double) " << var << " : " << asin_lut[i] << endl;
    // }

    glm::mat3x3 B = glm::transpose(glm::mat3x3{
        {0, 1, 2},
        {4, 5, 6},
        {8, 9, 10},
        });

    glm::mat3x3 C = glm::transpose(glm::mat3x3{
        {0, 1, 2},
        {3, 3, 3},
        {4, 5, 6},
    });

    // glm::acos( (double) x);
    printf("hello world!\n");
    cout << B << endl;
    cout << C << endl;
    cout << B*C << endl;

    cout << glm::pi<double>() << endl;
    cout << glm::mat4(1.0) << endl;
    glm::vec3 U(1,2,3);
    cout << C*U << endl;
    cout << U*C << endl;
    cout << C*glm::column(C, 0) << endl;
    return 0;
}