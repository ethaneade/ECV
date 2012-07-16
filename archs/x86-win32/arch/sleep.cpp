#include <ecv/timer.hpp>
#include <time.h>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

void ecv::sleep(double seconds)
{
    Sleep((DWORD)(seconds * 1e3 + 0.5));
}
