#pragma once

#include <cmath>
#include <assert.h>
#include <array>


template <typename T>
inline T clamp(T const& v, T const& vmin, T const& vmax)
{
    return v < vmin ? vmin : 
        v > vmax ? vmax : v;
}

inline double deg_to_rad(double deg) { return deg * M_PI / 180.; }

inline double rad_to_deg(double rad) { return rad * 180. / M_PI; }

template <typename T>
inline bool in_range(T const& v, T const& vmin, T const& vmax)
{
    return v >= vmin && v <= vmax;
}

template <typename T>
inline T square(T const& v)
{
    return v * v;
}

inline double round(double a, double step)
{
    assert(step > 0);
    return round(a / step) * step;
}

template <class Vec, class Fun, class T>
inline T aggregate(Vec const& vec, T const& init_val, Fun fun)
{
    T val = init_val;
    for (auto const& e : vec)
        val = fun(val, e);
    return val;
}

inline std::array<double,3> fitparabola(double x0, double y0, double x1, double y1, double x2, double y2)
{
    double x10 = x1 - x0;
    double x20 = x2 - x0;
    double x12 = x1 - x2;
    double y10 = y1 - y0;
    double y20 = y2 - y0;
    double y12 = y1 - y2;

    double a = (x20*y10 - x10*y20) / (x10*x20*x12);
    double b = (-x20*x20*y10 + x10*x10*y20) / (x10*x20*x12);
    double c = y0;

    double a_ = a;
    double b_ = -2*a*x0 + b;
    double c_ = a*x0*x0 - b*x0 + c;

    return {a_, b_, c_};
}
