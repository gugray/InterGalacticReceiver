#ifndef HARDWARE_CONTROLLER_H
#define HARDWARE_CONTROLLER_H

#include <pthread.h>
#include <stdint.h>
#include <vector>

struct InputReadings;

class HardwareController
{
  private:
    static const size_t buf_sz = 32;
    static uint8_t buffer[buf_sz];
    static int file_i2c;
    static pthread_t thread;
    static pthread_mutex_t mut;
    static std::vector<uint8_t> commands;
    static bool quitting;
    static int val_tuner;
    static int val_aknob;
    static int val_bknob;
    static int val_cknob;
    static int val_swtch;

  private:
    static void *loop(void *);
    static void deinit();
    static void process_values(const InputReadings &data);

  public:
    static bool init();
    static void exit();
    static void get_values(int &tuner, int &aknob, int &bknob, int &cknob, int &swtch);
    static void set_light(bool on);
};

int tuner_val_to_freq(int val);
int freq_to_tuner_val(int freq);

#endif
