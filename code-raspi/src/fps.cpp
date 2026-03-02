#include "fps.h"

#include <cstdio>
#include <math.h>
#include <unistd.h>

FPS::FPS(int target_fps)
    : target_fps(target_fps)
    , cycle_usec(1000000 / target_fps)
    , buf_size(target_fps)
    , ix(0)
{
    elapsec_usec = new long[buf_size];
    for (int i = 0; i < buf_size; ++i)
        elapsec_usec[i] = 0;
    gettimeofday(&ts_init, nullptr);
}

static long calc_elapsed_usec(const timeval &start, const timeval &end)
{
    long seconds = end.tv_sec - start.tv_sec;
    long microseconds = end.tv_usec - start.tv_usec;
    long total_microseconds = seconds * 1000000 + microseconds;
    return total_microseconds;
}

double FPS::frame_start()
{
    gettimeofday(&ts_start, nullptr);
    long usec_since_init = calc_elapsed_usec(ts_init, ts_start);
    double sec = usec_since_init / 1000000.0;
    return sec;
}

long FPS::get_avg_elapsed()
{
    int i = ix - 1;
    if (i < 0) i = buf_size - 1;
    long sum = 0, cnt = 0;
    while (i != ix && elapsec_usec[i] != 0)
    {
        sum += elapsec_usec[i];
        cnt += 1;
        i = i - 1;
        if (i < 0) i = buf_size - 1;
    }
    if (cnt == 0) return 0;
    return sum / cnt;
}

void FPS::frame_end()
{
    timeval ts_end;
    gettimeofday(&ts_end, nullptr);
    long elapsed = calc_elapsed_usec(ts_start, ts_end);

    long to_store = elapsed < cycle_usec ? cycle_usec : elapsed;
    elapsec_usec[ix] = to_store;
    ix = (ix + 1) % buf_size;

    int extrapolated_fps = round(1000000.0 / (double)elapsed);
    double elapsed_msec = (double)elapsed / 1000.0;

    long avg_elapsed = get_avg_elapsed();
    double avg_fps = 1000000.0 / (double)avg_elapsed;
    if (log_fps)
    {
        printf("FPS %5.1f / last frame %.2f msec (~%d FPS)    \r", avg_fps, elapsed_msec, extrapolated_fps);
    }

    if (elapsed >= cycle_usec) return;
    usleep(cycle_usec - elapsed);
}
