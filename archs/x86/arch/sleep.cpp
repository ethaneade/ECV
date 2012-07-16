#include <ecv/timer.hpp>
#include <time.h>

void ecv::sleep(double seconds)
{
    struct timespec ts;
    ts.tv_sec = (int)seconds;
    ts.tv_nsec = (seconds - ts.tv_sec) * 1e9;
    nanosleep(&ts, 0);
}
