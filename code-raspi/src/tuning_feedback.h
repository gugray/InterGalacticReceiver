#ifndef TUNING_FEEDBACK_H
#define TUNING_FEEDBACK_H

#include "tune_status.h"
#include <stdint.h>

class TuningFeedback
{
  private:
    const bool quiet;
    uint32_t last_changed_at = 0;
    TuneStatus prev_status = tsNone;
    bool is_pumming = false;

  private:
    static uint32_t get_msec();

  public:
    TuningFeedback(bool quiet);
    void tune_status(TuneStatus status);
};

#endif
