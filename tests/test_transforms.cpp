#include <iostream>
#include "../src/transforms.h"


int main()
{
    auto T = dh_transform(0.1, 0.2, 0.3, 0.4);
    Eigen::Matrix4d T1;
    T1 << 
        0.92106099,-0.3816559 ,  0.07736548,  0.0921061 ,
        0.38941834, 0.9027011 , -0.18298657,  0.03894183,
        0.        , 0.19866933,  0.98006658,  0.3       ,
        0.        , 0.        ,  0.        ,  1.        ;
    if (!mateq(T.matrix(), T1))
    {
        std::cerr << "dh transform failed!" << std::endl;
        return -1;
    }

    return 0;
}
