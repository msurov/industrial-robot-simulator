#pragma once

#include <ostream>
#include <sstream>
#include <array>
#include <vector>
#include <string>
#include "timeutils.h"


// simple value
template <typename T>
inline void format_value(std::ostream& s, T const& value)
{
    s << value;
}

// tuple
template <int i, int n, typename ... T>
struct format_tuple
{
    static void doit(std::ostream& s, std::tuple<T...> const& value)
    {
        int const i_next = i < n ? i + 1 : i;

        if (i == 0)
            s << "(" << std::get<i>(value);
        else if (i == n - 1)
            s << ", " << std::get<i>(value) << ")";
        else
            s << ", " << std::get<i>(value);

        format_tuple<i_next, n, T...>::doit(s, value);
    }
};

template <int n, typename ... T>
struct format_tuple<n,n,T...> {
    static void doit(std::ostream& s, std::tuple<T...> const& value) {}
};

template <typename ... T>
void format_value(std::ostream& s, std::tuple<T...> const& value)
{
    format_tuple<0, sizeof...(T), T...>::doit(s, value);
}

// vector
template <typename T>
void format_value(std::ostream& s, std::vector<T> const& arr)
{
    s << "[";
    for (std::size_t i = 0; i < arr.size() - 1; ++ i)
        s << arr[i] << ", ";
    if (arr.size() > 0)
        s << arr[arr.size() - 1];
    s << "]";
}

template <typename T, std::size_t N>
void format_value(std::ostream& s, std::array<T, N> const& arr)
{
    s << "[";
    for (std::size_t i = 0; i < arr.size() - 1; ++ i)
        s << arr[i] << ", ";
    if (arr.size() > 0)
        s << arr[arr.size() - 1];
    s << "]";
}

inline void __print(std::ostream& s)
{
}

template <class A1, class ... Atail> 
inline void __print(std::ostream& s, A1 const& a, Atail const& ... tail)
{
    format_value(s, a);
    __print(s, tail...);
}

inline std::string format_time(int64_t usec)
{
    char buf[64];
    sprintf(buf, "%0.6fsec", usec_to_sec(usec));
    return std::string(buf);
}

template <class ... Args>
inline std::string format_args(Args const& ... args)
{
    std::stringstream ss;
    __print(ss, args...);
    return ss.str();
}
