#include "matplotlibcpp.h"
#include "data_struct.h"
#include "QuadrupedRobotObject.h"

namespace plt = matplotlibcpp;

QuadrupedRobotObject OwO;

int main()
{
    OwO.foot_step = glm::vec3{50, 0 , 50};
    plt::plot({1,2,3,4}, "*");
    plt::show();
}