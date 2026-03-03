#include "main.h"

// Local dependencies
#include "error.h"
#include "file_helpers.h"
#include "hardware_controller.h"
#include "magic.h"
#include "tuner.h"
#include "tuning_feedback.h"

// Lib
#include "lib/canvas_ity.h"

// Global
#include <unistd.h>

static uint8_t *font_data;
static size_t font_data_size;
static Tuner tuner(true);

void test_tuner(bool quiet)
{
    TuningFeedback tfb(quiet);

    tuner.add_station(980);
    HardwareController::set_listeners(&tuner);
    HardwareController::init();

    canvas_ity::canvas ctx(W, H);
    float *image = new float[H * W * 4];
    char buf[64];

    font_data = load_canvas_font(&font_data_size);
    ctx.set_font(font_data, font_data_size, 64);
    free(font_data);
    font_data = nullptr;

    float cw = ctx.measure_text(" ");

    while (app_running)
    {
        usleep(100000);
        int vtuner, aknob, bknob, cknob, swtch;
        HardwareController::get_values(vtuner, aknob, bknob, cknob, swtch);
        float freq = Tuner::val_to_freq(vtuner) * 0.1;

        ctx.clear();
        ctx.set_color(canvas_ity::fill_style, 0.8, 0.8, 0.8, 1);
        sprintf(buf, "Tuner %5d", vtuner);
        ctx.fill_text(buf, 100, 100);
        sprintf(buf, "Freq  %5.1f", freq);
        ctx.fill_text(buf, 100, 164);

        const int vm = 4, hm = 46;
        ctx.set_line_width(2.0f);
        ctx.set_color(canvas_ity::stroke_style, 1, 0.3, 0.3, 1);
        ctx.stroke_rectangle(hm, vm, W - 2 * hm, H - 2 * vm);

        int station_ix;
        TuneStatus tuner_status;
        tuner.get_status(station_ix, tuner_status);

        tfb.tune_status(tuner_status);

        const float xmid = W / 2.0 - 1.5 * cw;
        if (tuner_status == tsNone)
        {
            ctx.set_color(canvas_ity::fill_style, 0.5, 0.5, 0.5, 1);
            ctx.fill_text("NNN", xmid, 300);
        }
        else
        {
            if (tuner_status == tsTuned)
                ctx.set_color(canvas_ity::fill_style, 0.2, 1.0, 0.2, 1);
            else if (tuner_status == tsAbove || tuner_status == tsBelow)
                ctx.set_color(canvas_ity::fill_style, 0.95, 0.65, 0.15, 1);
            else ctx.set_color(canvas_ity::fill_style, 0.8, 0.8, 0.8, 1);
            sprintf(buf, "S%02d", station_ix);
            ctx.fill_text(buf, xmid, 300);

            if (tuner_status == tsBelow) ctx.fill_text("<<", W / 2.0 + 2 * cw, 300);
            else if (tuner_status == tsAbove) ctx.fill_text(">>", W / 2.0 - 4 * cw, 300);
        }

        ctx.get_image_data(image, W, H);
        flush_to_fb(image);
    }
}
