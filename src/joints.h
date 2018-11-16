#pragma once

#include <array>
#include <assert.h>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include "math.h"


class Joints
{
public:
    static const int max_size = 8;

private:
    int _njoints;
    std::array<double, max_size> _joints;

public:

    Joints()
    {
        std::fill(_joints.begin(), _joints.end(), 0.);
        _njoints = 0;
    }

    Joints(Joints const& a) : 
        _njoints(a._njoints), _joints(a._joints)
    {
    }

    Joints(int n)
    {
        assert(n <= max_size);
        std::fill(_joints.begin(), _joints.end(), 0.);
        _njoints = n;
    }

    Joints(std::initializer_list<double> const& l)
    {
        assert(l.size() <= max_size);
        std::copy(l.begin(), l.end(), _joints.begin());
        _njoints = l.size();
    }

    inline void resize(int sz)
    {
        assert(sz <= max_size);
        _njoints = sz;
    }

    inline int size() const
    {
        return _njoints;
    }

    inline double& operator[] (int i)
    {
        return _joints[i];
    }

    inline double operator[] (int i) const
    {
        return _joints[i];
    }

    inline void fill(double const& val)
    {
        std::fill(_joints.begin(), _joints.begin() + _njoints, val);
    }

    inline void operator = (double const& val)
    {
        fill(val);
    }

    inline Joints mul(double k) const
    {
        const int n = size();
        Joints result(n);
        for (int i = 0; i < n; ++ i)
            result._joints[i] = _joints[i] * k;
        return result;
    }

    inline Joints div(double const& k) const
    {
        return mul(1/k);
    }

    inline Joints add(Joints const& a) const
    {
        const int n = a.size();
        assert(size() == n);
        Joints c(n);
        for (int i = 0; i < n; ++ i)
            c._joints[i] = _joints[i] + a._joints[i];
        return c;
    }

    inline Joints sub(Joints const& a) const
    {
        const int n = a.size();
        assert(size() == n);
        Joints c(n);
        for (int i = 0; i < n; ++ i)
            c._joints[i] = _joints[i] - a._joints[i];
        return c;
    }

    inline void operator += (Joints const& a)
    {
        const int n = a.size();
        assert(size() == n);
        for (int i = 0; i < n; ++ i)
            _joints[i] += a._joints[i];
    }

    inline void operator -= (Joints const& a)
    {
        const int n = a.size();
        assert(size() == n);
        for (int i = 0; i < n; ++ i)
            _joints[i] -= a._joints[i];
    }

    inline double L_inf_norm() const
    {
        return *std::max_element(_joints.begin(), _joints.begin() + _njoints);
    }

    static Joints from_vec(std::vector<double> const& vec)
    {
        const int n = vec.size();
        assert(n <= max_size);
        Joints joints(n);
        for (int i = 0; i < n; ++ i)
            joints[i] = vec[i];
        return joints;
    }

    static Joints zeros(int n)
    {
        Joints z(n);
        z.fill(0);
        return z;
    }

    inline int argmin() const
    {
        assert(_njoints > 0);
        auto i = std::min_element(_joints.begin(), _joints.begin() + _njoints);
        return std::distance(_joints.begin(), i);
    }

    inline int argmax() const
    {
        assert(_njoints > 0);
        auto i = std::max_element(_joints.begin(), _joints.begin() + _njoints);
        return std::distance(_joints.begin(), i);
    }

    inline double min() const
    {
        return _joints[argmin()];
    }

    inline double max() const
    {
        return _joints[argmax()];
    }

    inline Joints abs() const
    {
        Joints result(_njoints);

        for (int i = 0; i < _njoints; ++ i)
            result._joints[i] = std::fabs(_joints[i]);
        return result;
    }

    /*
     * elementwise multiply
     */
    inline Joints mulw(Joints const& a) const
    {
        assert(a.size() == _njoints);
        Joints result(_njoints);

        for (int i = 0; i < _njoints; ++ i)
            result._joints[i] = _joints[i] * a._joints[i];

        return result;
    }

    /*
     * elementwise divide
     */
    inline Joints divw(Joints const& a) const
    {
        assert(a.size() == _njoints);
        Joints result(_njoints);

        for (int i = 0; i < _njoints; ++ i)
            result._joints[i] = _joints[i] / a._joints[i];

        return result;
    }
};

inline Joints operator + (Joints const& a, Joints const& b) { return a.add(b); }
inline Joints operator - (Joints const& a, Joints const& b) { return a.sub(b); }
inline Joints operator * (Joints const& a, double k)  { return a.mul(k); }
inline Joints operator * (double k, Joints const& a)  { return a.mul(k); }
inline Joints operator / (Joints const& a, double k)  { return a.div(k); }

inline double L_inf_dist(Joints const& a, Joints const& b)
{
    const int n = a.size();
    assert(b.size() == n);

    double d = 0.;

    for (int i = 0; i < n; ++ i)
        d = std::max(d, std::fabs(a[i] - b[i]));
    return d;
}

inline bool operator == (Joints const& a, Joints const& b) { return L_inf_dist(a, b) < 1e-15; }
inline bool operator != (Joints const& a, Joints const& b) { return L_inf_dist(a, b) > 1e-15; }

inline double L_inf_norm(Joints const& a)
{
    return a.L_inf_norm();
}

inline std::string to_string(Joints const& joints)
{
    const int n = joints.size();
    std::stringstream ss;

    ss << "[";
    for (int i = 0; i < n; ++ i)
        ss << joints[i] << (i == n - 1 ? "]" : ", ");

    return ss.str();
}

inline std::ostream& operator << (std::ostream& s, Joints const& joints)
{
    const int n = joints.size();

    s << "[" << std::fixed << std::setprecision(10);
    for (int i = 0; i < n; ++ i)
    {
        s << joints[i];
        if (i < n-1)
            s << ", ";
    }

    s << "]";
    return s;
}

inline Joints max(Joints const& a, Joints const& b)
{
    const int n = a.size();
    assert(b.size() == n);
    Joints c;
    for (int j = 0; j < n; ++ j)
        c[j] = std::max(a[j], b[j]);
    return c;
}

inline Joints min(Joints const& a, Joints const& b)
{
    const int n = a.size();
    assert(b.size() == n);
    Joints c;
    for (int j = 0; j < n; ++ j)
        c[j] = std::min(a[j], b[j]);
    return c;
}

inline Joints round(Joints const& a, double step)
{
    const int n = a.size();
    Joints b(n);
    
    for (int i = 0; i < n; ++ i)
        b[i] = round(a[i], step);

    return b;
}

inline bool all_leq(Joints const& a, Joints const& b)
{
    const int n = a.size();
    assert(b.size() == n);

    for (int j = 0; j < n; ++ j)
    {
        if (!(a[j] <= b[j]))
            return false;
    }
    return true;
}

inline bool within(Joints const& q, Joints const& qmin, Joints const& qmax)
{
    return all_leq(qmin, q) && all_leq(q, qmax);
}
