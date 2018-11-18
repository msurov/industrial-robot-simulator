#include <iostream>
#include <tuple>
#include "../src/properties.h"
#include "../src/kinematics.h"


inline bool veceq(Vector3d const& a, Vector3d const& b, double eps=1e-10)
{
    return (a - b).lpNorm<Eigen::Infinity>() < eps;
}

bool test_ur5()
{
    using TestSampple = std::tuple<Joints,Vector3d>;
    std::initializer_list<TestSampple> testset{
        TestSampple{{0, 0, 0, 0, 0, 0}, {0.0, 0.19145, 1.00105}},
        TestSampple{{0, M_PI_2, 0, M_PI_2, 0, 0}, {0.81725,0.19145,-0.005491}},
        TestSampple{{1, 2 + M_PI_2, 3, -1 + M_PI_2, -2, -3}, {-0.033337, 0.0867089, 0.0840776}},
    };

    bool ok = true;

    for (auto const& sample : testset)
    {
        Joints q;
        Vector3d p;
        std::tie(q, p) = sample;
        auto T = kinematic_model::solvefk(q);
        if (!veceq(T.translation(), p, 1e-5))
        {
            std::cerr << "solve fk sample " << q << " failed: " << p.transpose() << " " << T.translation().transpose() << std::endl;
            ok = false;
        }
    }

    return ok;
}

bool test_ur10()
{
    Joints q = {0, -M_PI_2, M_PI_2, 0, 0, 0};
    auto T = kinematic_model::solvefk(q);
    std::cout << T << std::endl;
    return true;
}

int main(int argc, char const* argv[])
{
    bool ok = true;

    try
    {
        if (argc < 2)
        {
            std::cerr << "expect config as first arg" << std::endl;
            return -1;
        }

        properties::init(argv[1]);
        kinematic_model::init();
        // ok = test_ur5();
        test_ur10();
    }
    catch (std::exception const& e)
    {
        std::cerr << "failed: " << e.what() << std::endl;
        ok = false;
    }
    catch (...)
    {
        std::cerr << "failed: undef" << std::endl;
        ok = false;
    }

    return ok ? 0 : -1;
}
