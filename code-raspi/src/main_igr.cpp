#include "main.h"

// Local dependencies
#include "error.h"
#include "fps.h"
#include "hardware_controller.h"
#include "horrors.h"
#include "magic.h"
#include "render_blender.h"
#include "sketch_base.h"
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
static int sketch_ix = -1;

static void init_stations(GLuint render_fbo);
static void update_station(TuningFeedback &tfb, RenderBlender &renderer, double current_time);

void main_igr()
{
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

        update_station(tfb, renderer, current_time);
        if (sketch_ix == -1) continue;

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

void update_station(TuningFeedback &tfb, RenderBlender &renderer, double current_time)
{
    int station_ix;
    TuneStatus tuner_status;
    tuner.get_status(station_ix, tuner_status);

    tfb.tune_status(tuner_status);

    // DBG
    // station_ix = 5;
    // tuner_status = tsTuned;

    if (station_ix < -1) return;

    if (station_ix != sketch_ix && sketch_ix != -1)
    {
        sketches[sketch_ix]->unload(current_time);
        sketches[station_ix]->reload(current_time);
    }
    sketch_ix = station_ix;

    if (tuner_status == tsTuned)
        renderer.set_mode(bmSketch);
    else if (tuner_status == tsAbove || tuner_status == tsBelow)
        renderer.set_mode(bmInfo);
    else renderer.set_mode(bmStatic);
}
