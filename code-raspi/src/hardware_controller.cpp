#include "hardware_controller.h"

// Local dependencies
#include "error.h"
#include "lock.h"
#include "magic.h"
#include "value_listener_if.h"

// Global
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

uint8_t HardwareController::buffer[buf_sz] = {0};
int HardwareController::file_i2c = -1;
pthread_t HardwareController::thread;
pthread_mutex_t HardwareController::mut;
std::vector<uint8_t> HardwareController::commands;
bool HardwareController::quitting = false;
bool HardwareController::enabled = false;
bool HardwareController::mutex_ready = false;
int HardwareController::val_tuner;
int HardwareController::val_aknob;
int HardwareController::val_bknob;
int HardwareController::val_cknob;
int HardwareController::val_swtch;
IValueListener *HardwareController::tuner = nullptr;

static bool is_raspberry_pi()
{
    // On Raspberry Pi this file is usually present and contains a model string.
    const char *model_path = "/sys/firmware/devicetree/base/model";
    std::ifstream model_file(model_path);
    if (!model_file.good()) return false;

    std::string model;
    std::getline(model_file, model, '\0');
    return model.find("Raspberry Pi") != std::string::npos;
}

#pragma pack(push, 1)
struct InputReadings
{
    uint16_t tuner;
    uint16_t aKnob;
    uint16_t bKnob;
    uint16_t cKnob;
    uint8_t swtch;
};
#pragma pack(pop)

void HardwareController::init()
{
    quitting = false;
    enabled = false;
    bool on_raspi = is_raspberry_pi();
    if (!on_raspi)
    {
        printf("Hardware controller disabled: not running on Raspberry Pi.\n");
        return;
    }

    // Open I2C bus
    char *filename = (char *)I2C_NODE;
    if ((file_i2c = open(filename, O_RDWR)) < 0)
    {
        deinit();
        THROWF_ERRNO("Failed to open the I2C bus '%s'", filename);
    }

    // Access
    int addr = SLAVE_ADDRESS;
    if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
    {
        deinit();
        THROWF_ERRNO("Failed to acquire I2C bus access");
    }

    int r = pthread_mutex_init(&mut, NULL);
    if (r != 0)
    {
        deinit();
        THROWF("Failed to initialize mutex: %d: %s", r, strerror(r));
    }
    mutex_ready = true;

    // Start worker thread
    r = pthread_create(&thread, NULL, loop, NULL);
    if (r != 0)
    {
        deinit();
        THROWF("Failed to create thread: %d: %s", r, strerror(r));
    }
    pthread_detach(thread);
    enabled = true;
}

void HardwareController::deinit()
{
    try
    {
        if (file_i2c != -1) close(file_i2c);
        file_i2c = -1;
        mutex_ready = false;
        enabled = false;
    }
    catch (...)
    {
    }
}

void HardwareController::set_listeners(IValueListener *tuner)
{
    HardwareController::tuner = tuner;
}

void HardwareController::exit()
{
    quitting = true;
}

void *HardwareController::loop(void *)
{
    bool last_cycle_failed = false;
    InputReadings data;
    int length = (int)sizeof(InputReadings);

    // Sanity check to keep things in sync with MCU code
    if (length != 9)
    {
        fprintf(stderr, "Program error; giving up. Wrong size of InputReadings: %lu", sizeof(InputReadings));
        return nullptr;
    }

    while (!quitting)
    {
        usleep(HWCTRL_CYCLE_MSEC * 1000);

        // Send commands from queue
        while (true)
        {
            uint8_t cmd = 0xff;
            {
                Lock lock(&mut);
                if (commands.size() > 0)
                {
                    cmd = commands.front();
                    commands.erase(commands.begin());
                }
            }
            if (cmd == 0xff) break;
            buffer[0] = cmd;
            if (write(file_i2c, buffer, 1) != 1)
            {
                printf("Failed to write command to the I2C bus; it's lost now: %d: %s\n", errno, strerror(errno));
            }
        }

        // Send command 0x00 to read values
        buffer[0] = 0x00;
        if (write(file_i2c, buffer, 1) != 1)
        {
            if (!last_cycle_failed)
                printf("Failed to write 0x00 to the I2C bus: %d: %s\n", errno, strerror(errno));
            last_cycle_failed = true;
            continue;
        }
        else if (last_cycle_failed)
        {
            printf("Successful write to I2C bus after one or more failures.\n");
            last_cycle_failed = false;
        }

        // Read control values
        if (read(file_i2c, &data, length) != length)
        {
            if (!last_cycle_failed)
                fprintf(stderr, "Failed to read from the I2C bus: %d: %s\n", errno, strerror(errno));
            last_cycle_failed = true;
            continue;
        }
        else if (last_cycle_failed)
        {
            printf("Successful read from I2C bus after one or more failures.\n");
            last_cycle_failed = false;
        }

        // Smooth and store in thread-safe way
        process_values(data);
    }

    deinit();
    return nullptr;
}

void HardwareController::process_values(const InputReadings &data)
{
    if (data.aKnob >= 1024 || data.bKnob >= 1024 || data.bKnob >= 1024)
        return;
    if (data.swtch != 0 && data.swtch != 15)
        return;
    if (data.aKnob == 0 || data.bKnob == 0 || data.cKnob == 0 || data.tuner == 0)
        return;
    if (data.tuner >= 1024) return;
    // Discard nonsense values
    if (data.tuner > 100 && data.tuner < 924)
        __atomic_store_n(&val_tuner, (int)data.tuner, __ATOMIC_SEQ_CST);
    __atomic_store_n(&val_aknob, (int)data.aKnob, __ATOMIC_SEQ_CST);
    __atomic_store_n(&val_bknob, (int)data.bKnob, __ATOMIC_SEQ_CST);
    __atomic_store_n(&val_cknob, (int)data.cKnob, __ATOMIC_SEQ_CST);
    __atomic_store_n(&val_swtch, (int)data.swtch, __ATOMIC_SEQ_CST);
    // Update listeners with filtered values
    if (tuner != nullptr) tuner->update(val_tuner);
}

void HardwareController::get_values(int &tuner, int &aknob, int &bknob, int &cknob, int &swtch)
{
    tuner = __atomic_load_n(&val_tuner, __ATOMIC_SEQ_CST);
    aknob = __atomic_load_n(&val_aknob, __ATOMIC_SEQ_CST);
    bknob = __atomic_load_n(&val_bknob, __ATOMIC_SEQ_CST);
    cknob = __atomic_load_n(&val_cknob, __ATOMIC_SEQ_CST);
    swtch = __atomic_load_n(&val_swtch, __ATOMIC_SEQ_CST);
}

void HardwareController::set_light(bool on)
{
    if (!enabled || !mutex_ready) return;
    uint8_t cmd = on ? 0x11 : 0x10;
    Lock lock(&mut);
    commands.push_back(cmd);
}

void HardwareController::buzz(BuzzType bt)
{
    if (!enabled || !mutex_ready) return;
    Lock lock(&mut);
    if (bt == btBeepBeep) commands.push_back(0x20);
    else if (bt == btBoop) commands.push_back(0x21);
}

void HardwareController::set_led(LEDAction action)
{
    if (!enabled || !mutex_ready) return;
    Lock lock(&mut);
    if (action == laOff) commands.push_back(0x30);
    else if (action == laOn) commands.push_back(0x31);
    else if (action == laPum) commands.push_back(0x32);
    else if (action == laBlinkA) commands.push_back(0x33);
    else if (action == laBlinkB) commands.push_back(0x34);
}
