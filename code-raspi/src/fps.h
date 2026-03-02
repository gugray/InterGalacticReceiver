#ifndef FPS_H
#define FPS_H

#include <csignal>
#include <sys/time.h>

class FPS
{
  private:
    const int target_fps;
    const long cycle_usec;
    const int buf_size;
    long *elapsec_usec;
    int ix;
    timeval ts_init;
    timeval ts_start;

  public:
    bool log_fps = false;

  private:
    long get_avg_elapsed();

  public:
    FPS(int target_fps);
    double frame_start();
    void frame_end();
};

#endif
