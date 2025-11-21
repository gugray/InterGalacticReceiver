// Local dependencies
#include "canvas_ity.h"
#include "hardware_controller.h"

// Global
#include <algorithm>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <libgen.h>
#include <linux/fb.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#define SLAVE_ADDRESS 0x50
#define I2C_NODE "/dev/i2c-1"

static const size_t buf_sz = 4096;
static char buf[buf_sz];
static std::string bin_dir;
static const char *font_file_name = "IBMPlexMono-Regular.ttf";
static const char *fb_path = "/dev/fb0";
static int const width = 720, height = 576;

int flushToFB(float *image)
{
    int fb = open(fb_path, O_RDWR);
    if (fb < 0)
    {
        fprintf(stderr, "Failed to open '%s': %d: %s\n", fb_path, errno, strerror(errno));
        return 1;
    }

    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    ioctl(fb, FBIOGET_FSCREENINFO, &finfo);
    ioctl(fb, FBIOGET_VSCREENINFO, &vinfo);

    // printf("Framebuffer size: %d x %d; bits per pixel: %d\n", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);

    if (vinfo.xres != width || vinfo.yres != height)
    {
        fprintf(stderr, "Framebuffer resolution doesn't match image size of %d x %d\n", width, height);
        return 1;
    }
    if (vinfo.bits_per_pixel != 16)
    {
        fprintf(stderr, "Expected 16 bits per pixel, got %d\n", vinfo.bits_per_pixel);
        return 1;
    }

    long screensize = vinfo.yres_virtual * finfo.line_length;
    char *fbp = (char *)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fb, 0);
    if (fbp == MAP_FAILED)
    {
        fprintf(stderr, "Failed to map frame buffer to memory: %d: %s\n", errno, strerror(errno));
        return 1;
    }

    for (int y = 0; y < height; ++y)
    {
        int vofs = y * width * 4;
        for (int x = 0; x < width; ++x)
        {
            int ix = vofs + x * 4;
            float rf = image[ix];
            float gf = image[ix + 1];
            float bf = image[ix + 2];
            unsigned char r = static_cast<unsigned char>(rf * 31.0);
            unsigned char g = static_cast<unsigned char>(gf * 63.0);
            unsigned char b = static_cast<unsigned char>(bf * 31.0);
            int fb_pos = (x + vinfo.xoffset) * (vinfo.bits_per_pixel / 8) +
                         (y + vinfo.yoffset) * finfo.line_length;
            unsigned short color = (r << 11) | (g << 5) | (b);
            // color = 0xffff;
            // color = 0;
            *((unsigned short *)(fbp + fb_pos)) = color;
        }
    }

    munmap(fbp, screensize);
    close(fb);
    return 0;
}

unsigned char *load_file(const char *path, size_t *size_out)
{
    FILE *f = fopen(path, "rb");
    if (!f)
    {
        fprintf(stderr, "Failed to open '%s': %d: %s\n", path, errno, strerror(errno));
        return nullptr;
    }
    fseek(f, 0, SEEK_END);
    size_t size = ftell(f);
    rewind(f);

    unsigned char *buf = (unsigned char *)malloc(size);
    if (!buf)
    {
        fprintf(stderr, "Failed to allocated %lu bytes.\n", size);
        fclose(f);
        return nullptr;
    }

    fread(buf, 1, size, f);
    fclose(f);
    if (size_out) *size_out = size;
    return buf;
}

static const int rbsz = 4;
static int rbuf[rbsz] = {0};
static int rpos = 0;

int main()
{
    HardwareController::init();
    HardwareController::set_light(true);

    readlink("/proc/self/exe", buf, buf_sz - 1);
    dirname(buf);
    bin_dir.assign(buf);

    printf("Directory of executable: %s\n", bin_dir.c_str());

    canvas_ity::canvas ctx(width, height);
    float *image = new float[height * width * 4];
    char buf[64];

    size_t font_data_size;
    std::string font_path(bin_dir);
    font_path += "/";
    font_path += font_file_name;
    unsigned char *font_data = load_file(font_path.c_str(), &font_data_size);
    if (font_data == nullptr) return -1;
    ctx.set_font(font_data, font_data_size, 64);

    while (true)
    {
        int tuner, aknob, bknob, cknob, swtch;
        HardwareController::get_values(tuner, aknob, bknob, cknob, swtch);

        rbuf[rpos] = tuner;
        rpos = (rpos + 1) % rbsz;
        int sum = 0;
        for (int i = 0; i < rbsz; ++i)
            sum += rbuf[i];
        int avg = (sum + rbsz / 2) / rbsz;

        int freq = tuner_val_to_freq(tuner);

        // printf("\rTuner  %4d  SW %4d   ", tuner, swtch);

        usleep(10000);

        ctx.set_color(canvas_ity::fill_style, 0, 0, 0, 1);
        ctx.fill_rectangle(0, 0, width, height);

        ctx.set_line_width(6.0f);
        ctx.set_color(canvas_ity::stroke_style, 0.95, 0.65, 0.15, 1.0);
        ctx.begin_path();
        ctx.arc(360, 250, 70, 0, M_PI * 2);
        ctx.stroke();

        ctx.set_color(canvas_ity::fill_style, 0.8, 0.8, 0.8, 1);
        sprintf(buf, "Tuner %5d", tuner);
        ctx.fill_text(buf, 100, 100);
        sprintf(buf, "Freq  %5d", freq);
        ctx.fill_text(buf, 100, 164);

        sprintf(buf, "    A  %4d", aknob);
        ctx.fill_text(buf, 100, 228);
        sprintf(buf, "    B  %4d", bknob);
        ctx.fill_text(buf, 100, 292);
        sprintf(buf, "    C  %4d", cknob);
        ctx.fill_text(buf, 100, 356);

        ctx.get_image_data(image, width, height);
        flushToFB(image);
    }
}
