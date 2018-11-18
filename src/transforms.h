#pragma once
#include <iomanip>
#include <Eigen/Geometry>
#include <Eigen/Core>


using Eigen::Isometry3d;
using Eigen::AngleAxisd;
using Eigen::Vector3d;
using Eigen::Translation;


inline Isometry3d rotx(double a)
{
    return Isometry3d(AngleAxisd(a, Vector3d::UnitX()));
}

inline Isometry3d rotz(double a)
{
    return Isometry3d(AngleAxisd(a, Vector3d::UnitZ()));
}

inline Isometry3d trans(double x, double y, double z)
{
    return Isometry3d(Translation<double,3>(x, y, z));
}

inline Isometry3d rodrigues(double rx, double ry, double rz)
{
    Vector3d r {rx, ry, rz};
    double theta = r.norm();
    if (theta < 1e-8)
        return Isometry3d::Identity();
    AngleAxisd a(theta, r / theta);
    return Isometry3d(a);
}

inline Isometry3d maketransform(double x, double y, double z, double rx, double ry, double rz)
{
    return trans(x, y, z) * rodrigues(rx, ry, rz);
}

inline Isometry3d dh_transform(double r, double alpha, double d, double theta)
{
    auto const& Rx = rotx(alpha);
    auto const& Tx = trans(r, 0, 0);
    auto const& Rz = rotz(theta);
    auto const& Tz = trans(0, 0, d);
    auto const& T = Rz * Tz * Tx * Rx;
    return T;
}

constexpr double deg2rad(double deg)
{
    return deg * M_PI / 180;
}

constexpr double rad2deg(double rad)
{
    return rad * 180 / M_PI;
}

constexpr long double operator "" _deg(long double a)
{
    return deg2rad(a);
}

inline std::ostream& operator << (std::ostream& s, Isometry3d const& T)
{
    Vector3d t = T.translation();
    AngleAxisd a = AngleAxisd(T.linear());
    s << "[" << std::setprecision(5) << t.x() << "," << t.y() << "," << t.z() << "]" << " " 
      << rad2deg(a.angle()) << "° around [" << std::setprecision(3) << a.axis().x() << "," << a.axis().y() << "," << a.axis().z() << "]"; 
    return s;
}

inline bool mateq(Eigen::Matrix4d const& a, Eigen::Matrix4d const& b, double eps=1e-8)
{
    return (a - b).cwiseAbs().maxCoeff() < eps;
}

inline bool transeq(Isometry3d const& a, Isometry3d const& b, double eps=1e-8)
{
    return (a.matrix() - b.matrix()).cwiseAbs().maxCoeff() < eps;
}
