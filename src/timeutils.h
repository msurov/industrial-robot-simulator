#pragma once

#include <chrono>
#include <thread>
#include <unistd.h>


template <typename T>
inline int64_t sec_to_usec(T const& sec)
{
    return static_cast<int64_t>(sec * 1e+6);
}

inline double usec_to_sec(int64_t usec)
{
    return double(usec * 1e-6);
}

template <typename T>
inline int64_t msec_to_usec(T const& msec)
{
    return static_cast<int64_t>(msec * 1e+3);
}

inline int64_t epoch_usec()
{
    auto epoch = std::chrono::high_resolution_clock::now().time_since_epoch();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(epoch);
    return us.count();
}

inline void sleep_usec(int64_t usec)
{
    if (usec <= 0)
        return;
    usleep(usec);
}

template <typename T>
inline void sleep_sec(T const& sec)
{
    if (sec <= 0)
        return;
    sleep_usec(sec_to_usec(sec));
}

inline void sleep_until(int64_t t_end)
{
    while (true)
    {
        int64_t t = epoch_usec();
        int64_t duration = t_end - t;
        if (duration <= 0)
            return;

        sleep_usec(duration);
    }
}


class LoopRate
{
private:
    int64_t _t_prev;
    int64_t _interval;

public:
    LoopRate(int64_t interval_usec) : _t_prev(epoch_usec()), _interval(interval_usec) {}
    ~LoopRate() {}

    inline void wait()
    {
        int64_t t = epoch_usec();

        while (t < _t_prev + _interval)
        {
            sleep_usec(_t_prev + _interval - t);
            t = epoch_usec();
        }

        _t_prev = t;
    }
};
