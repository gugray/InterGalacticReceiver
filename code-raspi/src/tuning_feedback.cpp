#include "tuning_feedback.h"

// Local dependencies
#include "hardware_controller.h"
#include "magic.h"

// Global
#include <stdio.h>
#include <sys/time.h>

TuningFeedback::TuningFeedback()
{
}

uint32_t TuningFeedback::get_msec()
{
    timeval ts;
    gettimeofday(&ts, nullptr);
    long seconds = ts.tv_sec;
    long microseconds = ts.tv_usec;
    return seconds * 1000000 + microseconds;
}

void TuningFeedback::tune_status(TuneStatus status)
{
    // No change: we're done here
    if (status == prev_status) return;

    uint32_t now = get_msec();
    uint32_t elapsed = now - last_changed_at;

    // Going to tuned
    if (status == tsTuned)
    {
        // Boop buzz
        HardwareController::buzz(btBoop);
        HardwareController::set_led(laOn);
        is_pumming = false;
    }
    // Going to above/below
    else if (status == tsAbove || status == tsBelow)
    {
        // If we're entering station neighborhood from static, buzz
        // No buzz when we're leaving station, or if last change was less than a second ago
        if (prev_status != tsTuned && elapsed > 1000)
        {
            HardwareController::buzz(btBeepBeep);
        }
        HardwareController::set_led(status == tsAbove ? laBlinkA : laBlinkB);
        is_pumming = false;
    }
    else
    {
        HardwareController::set_led(laPum);
        is_pumming = true;
    }

    // Store change
    prev_status = status;
    last_changed_at = now;
}