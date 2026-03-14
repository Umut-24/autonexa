#ifdef USE_MICRO_ROS

#include <time.h>
#include <stdint.h>

#include "pico/time.h"

#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif

#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif

/*
 * libmicroros.a in this repo expects POSIX clock_gettime().
 * Provide a Pico-compatible shim backed by time_us_64().
 */
int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    (void)clk_id;
    if (tp == NULL) {
        return -1;
    }

    uint64_t us = time_us_64();
    tp->tv_sec = (time_t)(us / 1000000ULL);
    tp->tv_nsec = (long)((us % 1000000ULL) * 1000ULL);
    return 0;
}

#endif  /* USE_MICRO_ROS */
