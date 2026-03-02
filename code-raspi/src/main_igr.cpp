#include "main.h"

// Local dependencies
#include "error.h"
#include "fps.h"
#include "hardware_controller.h"
#include "horrors.h"
#include "info_overlay.h"
#include "magic.h"
#include "render_blender.h"
#include "sketch_base.h"
#include "sketch_info.h"
#include "tuner.h"
#include "tuning_feedback.h"

// Sketches
#include "sketches/anomaly/anomaly_sketch.h"
#include "sketches/bezix/bezix_sketch.h"
#include "sketches/cell/cell_sketch.h"
#include "sketches/mmgl01/mmgl01_sketch.h"
#include "sketches/ray/ray_sketch.h"
#include "sketches/star/star_sketch.h"

// Global
#include <vector>

static Tuner tuner(false);
static std::vector<SketchBase *> sketches;
static std::vector<int> freqs;
static int sketch_ix = -1;

static void init_stations(GLuint render_fbo);
static bool update_station(TuningFeedback &tfb, InfoOverlay &overlay, RenderBlender &renderer, double current_time);

void main_igr()
{
    InfoOverlay overlay(W, H);

    RenderBlender renderer;
    init_stations(renderer.fbo());

    HardwareController::set_listeners(&tuner);
    HardwareController::init();

    TuningFeedback tfb;

    FPS fps(TARGET_FPS);
    double last_time = fps.frame_start();
    fps.log_fps = true;

    HardwareController::set_light(true);

    while (app_running)
    {
        double current_time = fps.frame_start();
        double dt = current_time - last_time;
        last_time = current_time;

        bool render_sketch = update_station(tfb, overlay, renderer, current_time);
        if (sketch_ix == -1) continue;

        if (render_sketch)
            sketches[sketch_ix]->frame(dt);

        renderer.render(current_time);
        put_on_screen();
        fps.frame_end();

        int tuner, aknob, bknob, cknob, swtch;
        HardwareController::get_values(tuner, aknob, bknob, cknob, swtch);
    }

    HardwareController::set_light(false);
    HardwareController::set_led(laOff);
}

template <typename T>
void add_station(GLuint render_fbo, int freq)
{
    auto sketch = new T(W, H, render_fbo);
    sketch->init();
    tuner.add_station(freq);
    sketches.push_back(sketch);
    freqs.push_back(freq);
}

void init_stations(GLuint render_fbo)
{
    add_station<StarSketch>(render_fbo, 980);
    add_station<MMGL01Sketch>(render_fbo, 967);
    add_station<RaySketch>(render_fbo, 953);
    add_station<CellSketch>(render_fbo, 941);
    add_station<BezixSketch>(render_fbo, 932);
    add_station<AnomalySketch>(render_fbo, 920);
}

bool update_station(TuningFeedback &tfb, InfoOverlay &overlay, RenderBlender &renderer, double current_time)
{
    int new_ix;
    TuneStatus tuner_status;
    tuner.get_status(new_ix, tuner_status);

    tfb.tune_status(tuner_status);

    // DBG
    // new_ix = 5;
    // tuner_status = tsTuned;

    if (new_ix < -1) return false;

    // Unload previous sketch, load new one
    if (new_ix != sketch_ix && sketch_ix != -1)
    {
        sketches[sketch_ix]->unload(current_time);
        sketches[new_ix]->reload(current_time);
    }
    // If sketch changes: render overlay
    if (new_ix != sketch_ix)
    {
        // Render info overlay
        SketchInfo ski;
        ski.creator = "hamoid";
        ski.title = "UN3091";
        const uint8_t *px_data = overlay.render(ski, freqs[new_ix]);
        // Give image to renderer
        renderer.set_overlay(px_data);
    }
    sketch_ix = new_ix;

    bool render_sketch = true;
    if (tuner_status == tsTuned)
        renderer.set_mode(bmSketch);
    else if (tuner_status == tsAbove || tuner_status == tsBelow)
        renderer.set_mode(bmInfo);
    else
    {
        render_sketch = false;
        renderer.set_mode(bmStatic);
    }

    return render_sketch;
}
