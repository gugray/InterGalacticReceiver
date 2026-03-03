#include "info_overlay.h"

// Local dependencies
#include "file_helpers.h"
#include "sketch_info.h"

// Global
#include <math.h>

static const char *font_file_name = "Anta-Regular.ttf";
static const float sz_title = 56;
static const float sz_author = 48;
static const float sz_freq = 44;

InfoOverlay::InfoOverlay(int w, int h)
    : w(w)
    , h(h)
    , pixels(new uint8_t[w * h * 4])
    , image(new float[w * h * 4])
    , ctx(w, h)
{
    // Load font
    std::string font_path_full;
    path_from_bindir(font_file_name, font_path_full);
    font_data = load_file(font_path_full.c_str(), &font_data_size);
}

uint8_t const *InfoOverlay::render(const SketchInfo &info, uint16_t freq)
{
    ctx.global_composite_operation = canvas_ity::source_copy;
    ctx.set_color(canvas_ity::fill_style, 0, 0, 0, 0.8);
    ctx.fill_rectangle(0, 0, w, h);
    ctx.global_composite_operation = canvas_ity::source_over;

    float y = 24;
    // IGR amber brand color: #e1b01b => 0.882, 0.69, 0.106
    ctx.set_color(canvas_ity::fill_style, 0.882, 0.69, 0.106, 1);

    // Title Line 1
    ctx.set_font(font_data, font_data_size, sz_title);
    y += sz_title;
    snprintf(buf, buf_sz, "%s", info.title1.c_str());
    ctx.fill_text(buf, 80, y);
    // (Optional) Title Line 2
    if (!info.title2.empty())
    {
        y += sz_title * 0.9;
        snprintf(buf, buf_sz, "%s", info.title2.c_str());
        ctx.fill_text(buf, 80, y);
    }
    ctx.set_font(font_data, font_data_size, sz_author);
    snprintf(buf, buf_sz, "from %s", info.creator.c_str());
    y += sz_title * 1.2;
    ctx.fill_text(buf, 80, y);

    // Green color matching amber
    ctx.set_color(canvas_ity::fill_style, 0.404, 0.816, 0.522, 1);
    ctx.set_font(font_data, font_data_size, sz_freq);
    snprintf(buf, buf_sz, "%d.%d QHz", freq / 10, freq % 10);
    float freq_width = ctx.measure_text(buf);
    ctx.fill_text(buf, w - freq_width - 80, h - 40);

    ctx.get_image_data(image, w, h);
    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            int ix = (y * w + x) * 4;
            float fr = image[ix];
            float fg = image[ix + 1];
            float fb = image[ix + 2];
            float fa = image[ix + 3];
            ix = ((h - y - 1) * w + x) * 4;
            pixels[ix] = round(fr * 255.0);
            pixels[ix + 1] = round(fg * 255.0);
            pixels[ix + 2] = round(fb * 255.0);
            pixels[ix + 3] = round(fa * 255.0);
        }
    }
    return pixels;
}