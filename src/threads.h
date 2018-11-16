#pragma once

#include <sys/mman.h>
#include <thread>
#include "timeutils.h"


inline bool set_thread_rt_priotiy(pthread_t thread, int priotity)
{
    // TODO: mlockall(MCL_CURRENT | MCL_FUTURE);

    if (thread == pthread_t(-1))
        thread = pthread_self();

    int policy = SCHED_RR; // SCHED_FIFO;
    sched_param param;
    param.sched_priority = priotity;
    return pthread_setschedparam(thread, policy, &param) == 0;
}
