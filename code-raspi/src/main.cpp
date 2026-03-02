#include "main.h"

// Local dependencies
#include "arg_parse.h"
#include "error.h"
#include "file_helpers.h"
#include "horrors.h"
#include "magic.h"

// Global
#include <csignal>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

static const char *font_file_name = "IBMPlexMono-Regular.ttf";

bool app_running = true;

// clang-format off
#define ACT_CALIBRATE   "action_calibrate"
#define ACT_TUNER       "action_test_tuner"
#define ACT_RUN         "action_run"
// clang-format on

static std::string device_path;
static std::string action;

static void sighandler(int);
static bool parse_args(int argc, const char *argv[]);

int main(int argc, const char *argv[])
{

    try
    {
        signal(SIGINT, sighandler);
        signal(SIGTERM, sighandler);

        if (!parse_args(argc, argv)) return -1;

        if (action == ACT_CALIBRATE) calibrate_readings();
        else if (action == ACT_TUNER) test_tuner();
        else if (action == ACT_RUN)
        {
            if (should_use_drm_backend())
            {
                if (device_path.empty())
                {
                    const char *found_device = find_display_device();
                    if (found_device == nullptr) THROWF("No connected display device found");
                    device_path.assign(found_device);
                    delete[] found_device;
                }
            }
            else if (!device_path.empty())
            {
                printf("Ignoring --dev option in desktop mode.\n");
                device_path.clear();
            }

            init_horrors(device_path.c_str());
            main_igr();
            cleanup_horrors();
        }
        printf("\nGoodbye!\n");
        return 0;
    }
    catch (const igr_exception &e)
    {
        fprintf(stderr, "Runtime error in file %s line %d: %s:\n%s\n", e.file(), e.line(), e.func(), e.what());
        cleanup_horrors();
        return -1;
    }
    catch (const std::bad_alloc &e)
    {
        fprintf(stderr, "Out of memory: %s\n", e.what());
        cleanup_horrors();
        return -1;
    }
    catch (...)
    {
        fprintf(stderr, "Unexpected error\n");
        cleanup_horrors();
        return -1;
    }
}

static void sighandler(int)
{
    app_running = false;
}

static bool parse_args(int argc, const char *argv[])
{
    auto parser = ArgumentParser("igr");
    parser.add_argument(ACT_CALIBRATE, "calib", "calibrate-readings", "Action: Calibrate readings");
    parser.add_argument(ACT_TUNER, "tuner", "test-tuner", "Action: Test tuner");
    parser.add_argument(ACT_RUN, "run", "", "Action: Run normally with sketches");
    parser.add_argument("help", "--help", "", "Displays this help message");
    parser.add_argument("dev", "", "--dev", "Device path (default: /dev/dri/card0)", STORE);

    bool success = parser.parse(argv, argc, stdout);
    if (!success || parser.get("help").is_set)
    {
        parser.print_usage(stdout);
        exit(success ? 0 : 1);
    }

    bool ok = true;
    bool multiple_actions = false;

    if (parser.get(ACT_CALIBRATE).is_set) action = ACT_CALIBRATE;
    if (parser.get(ACT_TUNER).is_set)
    {
        if (!action.empty()) multiple_actions = true;
        else action = ACT_TUNER;
    }
    if (parser.get(ACT_RUN).is_set)
    {
        if (!action.empty()) multiple_actions = true;
        else action = ACT_RUN;
    }

    if (multiple_actions || action.empty())
    {
        parser.print_usage(stdout);
        printf("\nBad arguments: Exactly one action must be specified\n");
        return false;
    }

    if (parser.get("dev").is_set) device_path.assign(parser.get("dev").value.c_str());

    if (!ok)
    {
        parser.print_usage(stdout);
        return false;
    }
    else return true;
}

void flush_to_fb(float *image)
{
    int fb = open(FB_PATH, O_RDWR);
    if (fb < 0)
        THROWF_ERRNO("Failed to open '%s'", FB_PATH);

    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    ioctl(fb, FBIOGET_FSCREENINFO, &finfo);
    ioctl(fb, FBIOGET_VSCREENINFO, &vinfo);

    // printf("Framebuffer size: %d x %d; bits per pixel: %d\n", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);

    if (vinfo.xres != W || vinfo.yres != H)
        THROWF("Framebuffer resolution (%d x %d) doesn't match image size of %d x %d", vinfo.xres, vinfo.yres, W, H);

    if (vinfo.bits_per_pixel != 16)
        THROWF("Expected 16 bits per pixel, got %d", vinfo.bits_per_pixel);

    long screensize = vinfo.yres_virtual * finfo.line_length;
    char *fbp = (char *)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fb, 0);
    if (fbp == MAP_FAILED)
        THROWF_ERRNO("Failed to map frame buffer to memory");

    for (int y = 0; y < H; ++y)
    {
        int vofs = y * W * 4;
        for (int x = 0; x < W; ++x)
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
}

uint8_t *load_canvas_font(size_t *data_size)
{
    std::string font_path_full;
    path_from_bindir(font_file_name, font_path_full);
    return load_file(font_path_full.c_str(), data_size);
}
