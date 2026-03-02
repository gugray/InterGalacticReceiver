#include "horrors.h"

// Local dependencies
#include "error.h"
#include "main.h"
#include "magic.h"

// Global
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <unistd.h>
#if HAS_SDL2
#include <SDL2/SDL.h>
#endif

int drm_fd = -1;
drmModeRes *resources = nullptr;
drmModeConnectorPtr conn = nullptr;
drmModeEncoderPtr enc = nullptr;
drmModeCrtc *saved_crtc = nullptr;
drmModeModeInfo mode;
gbm_device *gbm_dev = nullptr;
gbm_surface *gbm_surf = nullptr;
EGLDisplay egl_display = EGL_NO_DISPLAY;
EGLContext egl_ctx = EGL_NO_CONTEXT;
EGLSurface egl_surf = EGL_NO_SURFACE;
gbm_bo *bo = nullptr;
uint32_t fb_id = 0;
uint32_t prev_fb_id = 0;
bool kms_scanout_enabled = true;
bool use_sdl_window = false;
#if HAS_SDL2
static SDL_Window *sdl_window = nullptr;
static SDL_GLContext sdl_gl_ctx = nullptr;
#endif

static bool is_raspberry_pi()
{
    std::ifstream model_file("/sys/firmware/devicetree/base/model");
    if (!model_file.good()) model_file.open("/proc/device-tree/model");
    if (!model_file.good()) return false;
    std::string model;
    std::getline(model_file, model, '\0');
    return model.find("Raspberry Pi") != std::string::npos;
}

bool should_use_drm_backend()
{
#if !HAS_SDL2
    // If SDL is not compiled in, DRM is the only available backend.
    return true;
#endif
    return is_raspberry_pi();
}

void cleanup_horrors()
{
    if (use_sdl_window)
    {
#if HAS_SDL2
        if (sdl_gl_ctx != nullptr)
        {
            SDL_GL_DeleteContext(sdl_gl_ctx);
            sdl_gl_ctx = nullptr;
        }
        if (sdl_window != nullptr)
        {
            SDL_DestroyWindow(sdl_window);
            sdl_window = nullptr;
        }
        SDL_QuitSubSystem(SDL_INIT_VIDEO);
#endif
        use_sdl_window = false;
    }

    if (bo)
    {
        // Remove fb and release BO
        if (fb_id) drmModeRmFB(drm_fd, fb_id);
        gbm_surface_release_buffer(gbm_surf, bo);
    }

    // Uninit EGL
    if (egl_display != EGL_NO_DISPLAY)
    {
        eglMakeCurrent(egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (egl_ctx != EGL_NO_CONTEXT) eglDestroyContext(egl_display, egl_ctx);
        if (egl_surf != EGL_NO_SURFACE) eglDestroySurface(egl_display, egl_surf);
        eglTerminate(egl_display);
    }

    if (gbm_surf) gbm_surface_destroy(gbm_surf);
    if (gbm_dev) gbm_device_destroy(gbm_dev);

    // Restore saved CRTC if we saved one
    if (saved_crtc)
    {
        drmModeSetCrtc(drm_fd, saved_crtc->crtc_id, saved_crtc->buffer_id,
                       saved_crtc->x, saved_crtc->y, &conn->connector_id, 1, &saved_crtc->mode);
        drmModeFreeCrtc(saved_crtc);
    }

    if (enc) drmModeFreeEncoder(enc);
    if (conn) drmModeFreeConnector(conn);
    if (resources != nullptr) drmModeFreeResources(resources);
    if (drm_fd >= 0) close(drm_fd);
}

static void list_connectors()
{
    fprintf(stderr, "Available connectors:\n");
    for (int i = 0; i < resources->count_connectors; ++i)
    {
        drmModeConnectorPtr c = drmModeGetConnector(drm_fd, resources->connectors[i]);
        if (!c) continue;
        printf("ID: %u type: %u connection: %d modes: %d\n",
               c->connector_id, c->connector_type, c->connection, c->count_modes);
        drmModeFreeConnector(c);
    }
}

static drmModeConnectorPtr get_preferred_connector()
{
    list_connectors();

    // Prefer composite connector, but will use first OK one otherwise
    drmModeConnectorPtr first_connector = nullptr;
    drmModeConnectorPtr composite_connector = nullptr;

    for (int i = 0; i < resources->count_connectors; ++i)
    {
        uint32_t conn_id = resources->connectors[i];
        drmModeConnectorPtr c = drmModeGetConnector(drm_fd, conn_id);
        if (!c) continue;
        if (c->count_modes == 0)
        {
            drmModeFreeConnector(c);
            continue;
        }
        if (composite_connector == nullptr && c->connector_type == DRM_MODE_CONNECTOR_Composite)
            composite_connector = c;
        else if (first_connector == nullptr)
            first_connector = c;
        else drmModeFreeConnector(c);
    }
    if (composite_connector != nullptr)
    {
        if (first_connector != nullptr) drmModeFreeConnector(first_connector);
        return composite_connector;
    }
    if (first_connector != nullptr)
    {
        if (composite_connector != nullptr) drmModeFreeConnector(composite_connector);
        return first_connector;
    }

    THROWF("No suitable DRM connector found");
    return nullptr; // Shut up compiler
}

static drmModeModeInfo get_first_or_preferred_mode()
{
    drmModeModeInfo mode = conn->modes[0];
    for (int i = 0; i < conn->count_modes; ++i)
    {
        if (conn->modes[i].type & DRM_MODE_TYPE_PREFERRED)
        {
            mode = conn->modes[i];
            break;
        }
    }
    printf("Using connector %u mode '%s' %ux%u\n", conn->connector_id, mode.name, mode.hdisplay, mode.vdisplay);
    return mode;
}

static void init_egl()
{
    egl_display = eglGetDisplay((EGLNativeDisplayType)gbm_dev);
    if (egl_display == EGL_NO_DISPLAY)
        THROWF("eglGetDisplay failed: %d", eglGetError());
    if (!eglInitialize(egl_display, nullptr, nullptr))
        THROWF("eglInitialize failed: %d", eglGetError());

    EGLint cfg_attribs[] = {
        EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
        EGL_RED_SIZE, 8,
        EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8,
        EGL_ALPHA_SIZE, 8,
        EGL_DEPTH_SIZE, 24,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_NONE};
    EGLConfig cfg;
    EGLint num_cfg;
    if (!eglChooseConfig(egl_display, cfg_attribs, &cfg, 1, &num_cfg) || num_cfg < 1)
        THROWF("eglChooseConfig failed: %d", eglGetError());

    EGLint ctx_attribs[] = {EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE};
    egl_ctx = eglCreateContext(egl_display, cfg, EGL_NO_CONTEXT, ctx_attribs);
    if (egl_ctx == EGL_NO_CONTEXT) THROWF("eglCreateContext failed: %d", eglGetError());

    egl_surf = eglCreateWindowSurface(egl_display, cfg, (EGLNativeWindowType)gbm_surf, nullptr);
    if (egl_surf == EGL_NO_SURFACE) THROWF("eglCreateWindowSurface failed: %d", eglGetError());

    if (!eglMakeCurrent(egl_display, egl_surf, egl_surf, egl_ctx))
        THROWF("eglMakeCurrent failed: %d", eglGetError());
}

static bool set_crtc(drmModeModeInfo mode, uint32_t fb_id)
{
    uint32_t crtc_id = 0;
    // Prefer encoder's crtc, else first available
    if (enc && enc->crtc_id) crtc_id = enc->crtc_id;
    else if (resources->count_crtcs > 0) crtc_id = resources->crtcs[0];

    if (!crtc_id) THROWF("No available CRTC");

    int ret = drmModeSetCrtc(drm_fd, crtc_id, fb_id, 0, 0, &conn->connector_id, 1, &mode);
    if (ret)
    {
        if (errno == EACCES || errno == EPERM)
        {
            fprintf(stderr,
                    "drmModeSetCrtc denied (%d: %s). Continuing without KMS scanout.\n",
                    errno, strerror(errno));
            return false;
        }
        THROWF_ERRNO("drmModeSetCrtc failed");
    }
    return true;
}

#if HAS_SDL2
static void init_sdl_window()
{
    if (SDL_Init(SDL_INIT_VIDEO) != 0)
        THROWF("SDL_Init failed: %s", SDL_GetError());

    if (SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES) != 0 ||
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2) != 0 ||
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0) != 0 ||
        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1) != 0)
    {
        THROWF("SDL_GL_SetAttribute failed: %s", SDL_GetError());
    }

    sdl_window = SDL_CreateWindow("igr",
                                  SDL_WINDOWPOS_CENTERED,
                                  SDL_WINDOWPOS_CENTERED,
                                  W,
                                  H,
                                  SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    if (sdl_window == nullptr)
        THROWF("SDL_CreateWindow failed: %s", SDL_GetError());

    sdl_gl_ctx = SDL_GL_CreateContext(sdl_window);
    if (sdl_gl_ctx == nullptr)
        THROWF("SDL_GL_CreateContext failed: %s", SDL_GetError());

    SDL_GL_SetSwapInterval(1);
    use_sdl_window = true;
    printf("Using SDL2 window backend (%dx%d).\n", W, H);
}
#endif

void init_horrors(const char *device_path)
{
    if (!should_use_drm_backend())
    {
#if HAS_SDL2
        init_sdl_window();
        return;
#else
        THROWF("Desktop output requires SDL2, but this binary was built without SDL2 support");
#endif
    }

    kms_scanout_enabled = true;
    printf("Initializing video device: %s\n", device_path);
    drm_fd = open(device_path, O_RDWR | O_CLOEXEC);
    if (drm_fd < 0) THROWF_ERRNO("Failed to open device '%s'", device_path);

    resources = drmModeGetResources(drm_fd);
    if (!resources) THROWF_ERRNO("drmModeGetResources failed");

    conn = get_preferred_connector();
    if (conn->encoder_id) enc = drmModeGetEncoder(drm_fd, conn->encoder_id);

    // Save current CRTC (if any) so we can restore later
    if (enc && enc->crtc_id) saved_crtc = drmModeGetCrtc(drm_fd, enc->crtc_id);

    // Pick preferred or first mode
    mode = get_first_or_preferred_mode();

    // GBM device and surface
    gbm_dev = gbm_create_device(drm_fd);
    if (!gbm_dev) THROWF("gbm_create_device failed");

    gbm_surf = gbm_surface_create(gbm_dev,
                                  mode.hdisplay,
                                  mode.vdisplay,
                                  GBM_FORMAT_XRGB8888,
                                  GBM_BO_USE_SCANOUT | GBM_BO_USE_RENDERING);
    if (!gbm_surf) THROWF("gbm_surface_create failed");

    // EGL init
    init_egl();
}

static bool card_has_active_connector(const char *card)
{
    int fd = open(card, O_RDWR | O_CLOEXEC);
    if (fd < 0) return false;

    drmModeRes *res = drmModeGetResources(fd);
    if (!res)
    {
        close(fd);
        return false;
    }

    bool ok = false;
    for (int i = 0; !ok && i < res->count_connectors; i++)
    {
        drmModeConnector *conn = drmModeGetConnector(fd, res->connectors[i]);
        if (!conn) continue;
        if ((conn->connection == DRM_MODE_CONNECTED || conn->connection == DRM_MODE_UNKNOWNCONNECTION) &&
            conn->count_modes > 0)
        {
            ok = true;
        }
        drmModeFreeConnector(conn);
    }
    drmModeFreeResources(res);
    close(fd);
    return ok;
}

char *find_display_device()
{
    const char *dri_dir = "/dev/dri";
    DIR *dir = opendir(dri_dir);
    if (!dir) return nullptr;

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL)
    {
        if (strncmp(entry->d_name, "card", 4) != 0) continue;
        std::string path = std::string(dri_dir) + "/" + entry->d_name;
        if (card_has_active_connector(path.c_str()))
        {
            char *res = new char[path.size() + 1];
            strcpy(res, path.c_str());
            closedir(dir);
            return res;
        }
    }
    closedir(dir);
    return nullptr;
}

void put_on_screen()
{
    if (use_sdl_window)
    {
#if HAS_SDL2
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT) app_running = false;
        }
        SDL_GL_SwapWindow(sdl_window);
        return;
#endif
    }

    // Swap EGL buffers
    if (!eglSwapBuffers(egl_display, egl_surf))
        THROWF("eglSwapBuffers failed: %d", eglGetError());

    if (!kms_scanout_enabled) return;

    // Get new buffer
    gbm_bo *new_bo = gbm_surface_lock_front_buffer(gbm_surf);
    if (!new_bo) THROWF("gbm_surface_lock_front_buffer failed");

    uint32_t width = gbm_bo_get_width(new_bo);
    uint32_t height = gbm_bo_get_height(new_bo);
    uint32_t stride = gbm_bo_get_stride(new_bo);
    uint32_t handle = gbm_bo_get_handle(new_bo).u32;

    uint32_t handles[4] = {handle, 0, 0, 0};
    uint32_t pitches[4] = {stride, 0, 0, 0};
    uint32_t offsets[4] = {0, 0, 0, 0};

    uint32_t new_fb_id = 0;
    if (drmModeAddFB2(drm_fd, width, height, GBM_FORMAT_XRGB8888,
                      handles, pitches, offsets, &new_fb_id, 0))
    {
        THROWF_ERRNO("drmModeAddFB2 failed");
    }

    // Set new framebuffer before releasing old buffer
    if (!set_crtc(mode, new_fb_id))
    {
        // Keep app running without direct KMS output.
        kms_scanout_enabled = false;
        drmModeRmFB(drm_fd, new_fb_id);
        gbm_surface_release_buffer(gbm_surf, new_bo);
        return;
    }

    // Now safe to release old buffer and remove old FB
    if (bo) gbm_surface_release_buffer(gbm_surf, bo);
    if (prev_fb_id) drmModeRmFB(drm_fd, prev_fb_id);

    // Update for next iteration
    bo = new_bo;
    prev_fb_id = fb_id;
    fb_id = new_fb_id;
}
