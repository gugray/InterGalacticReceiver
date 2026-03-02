#include "info_overlay.h"

// Local dependencies
#include "file_helpers.h"
#include "sketch_info.h"

// Global
#include <math.h>

static const char *font_file_name = "Anta-Regular.ttf";

InfoOverlay::InfoOverlay(int w, int h)
    : w(w)
    , h(h)
    , pixels(new uint8_t[w * h * 4])
    , image(new float[w * h * 4])
    , ctx(w, h)
{
    load_font();
}

void InfoOverlay::load_font()
{
    std::string font_path_full;
    path_from_bindir(font_file_name, font_path_full);
    size_t font_data_size;
    uint8_t *font_data = load_file(font_path_full.c_str(), &font_data_size);
    ctx.set_font(font_data, font_data_size, 64);
    free(font_data);
}

uint8_t const *InfoOverlay::render(const SketchInfo &info, uint16_t freq)
{
    ctx.clear();
    ctx.set_color(canvas_ity::fill_style, 0.8, 0.2, 0.2, 1);
    snprintf(buf, buf_sz, "Vlhurg %5d", 785);
    ctx.fill_text(buf, 100, 100);

    ctx.get_image_data(image, w, h);
    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            int ix = (y * w + x) * 4;
            float fr = image[ix];
            float fg = image[ix + 1];
            float fb = image[ix + 2];
            ix = ((h - y - 1) * w + x) * 4;
            pixels[ix] = round(fr * 255.0);
            pixels[ix + 1] = round(fg * 255.0);
            pixels[ix + 2] = round(fb * 255.0);
            pixels[ix + 3] = 255;
        }
    }
    return pixels;
}