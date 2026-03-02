#include <Arduino.h>
#include <Wire.h>

#include "buffer_log.h"
#include "magic.h"

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

static const int recvBufSz = 8;
static const int sendBufSz = 16;

static volatile uint8_t recvBuf[recvBufSz];
static volatile int recvBufPtr = 0;
static volatile uint8_t sendBuf[sendBufSz];
static volatile int sendBytes = 0;
static volatile BufferLog aKnobLog;
static volatile BufferLog bKnobLog;
static volatile BufferLog cKnobLog;
static volatile BufferLog tunerLog;
static volatile uint8_t swtch;

// When last vibe action started
static volatile uint32_t vibe_start = 0;
// 0: no vibe; 1: beepbeep; 2: boop
static volatile uint8_t vibe_action = 0;

// 0: off; 1: both on; 2: both pum pum; 3: blink A; 4: blink B
static volatile uint8_t led_action = 3;
// counter with different meanings per action
static volatile uint16_t led_counter = 0;
static volatile uint16_t led_duty = 0;

static void onWireReceive(int bytecount);
static void onWireRequest();
static void handleCommand(uint8_t b);

void setup()
{
    // Onboard LED
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    digitalWrite(ONBOARD_LED_PIN, HIGH);

    // Device inputs
    analogReference(VDD);
    pinMode(KNOB_A_PIN, INPUT);
    pinMode(KNOB_B_PIN, INPUT);
    pinMode(KNOB_C_PIN, INPUT);
    pinMode(TUNER_PIN, INPUT);
    pinMode(KNOB_SWITCH_PIN, INPUT);

    // Device outputs
    pinMode(LIGHT_CTRL_PIN, OUTPUT);
    pinMode(VIBE_CTRL_PIN, OUTPUT);
    pinMode(LEDA_PIN, OUTPUT);
    digitalWrite(LEDA_PIN, HIGH);
    pinMode(LEDB_PIN, OUTPUT);
    digitalWrite(LEDB_PIN, HIGH);

    // CPU clock 10MHz: prescaler enabled, DIV2
    CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm;

    // Timer B0, MEASURE_FREQ times per second (500)
    TCB0.CTRLA = 0;
    TCB0.CTRLB = TCB_CNTMODE_INT_gc;
    TCB0.CCMP = (10000000UL / MEASURE_FREQ) - 1;
    TCB0.INTCTRL = TCB_CAPT_bm;
    TCB0.CTRLA = TCB_ENABLE_bm;

    // Timer B9, PWM_FREQ for LED blinking
    TCB1.CTRLA = 0;
    TCB1.CTRLB = TCB_CNTMODE_INT_gc;
    TCB1.CCMP = (10000000UL / PWM_FREQ) - 1;
    TCB1.INTCTRL = TCB_CAPT_bm;
    TCB1.CTRLA = TCB_ENABLE_bm | TCB_CLKSEL_DIV1_gc;

    // I2C slave
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(onWireReceive);
    Wire.onRequest(onWireRequest);
}

static uint32_t tick_counter = 0;

ISR(TCB0_INT_vect)
{
    TCB0.INTFLAGS = TCB_CAPT_bm;

    aKnobLog.logValue(analogRead(KNOB_A_PIN));
    bKnobLog.logValue(analogRead(KNOB_B_PIN));
    cKnobLog.logValue(analogRead(KNOB_C_PIN));
    tunerLog.logValue(analogRead(TUNER_PIN));

    pinMode(KNOB_SWITCH_PIN, INPUT);
    if (digitalRead(KNOB_SWITCH_PIN) == LOW) swtch = 0;
    else swtch = 15;

    // beep beep
    if (vibe_action == 1)
    {
        uint16_t elapsed = (uint16_t)(tick_counter - vibe_start);
        if (elapsed < BEEP_TICKS || (elapsed > BEEP_TICKS + BEEP_DELAY && elapsed < 2 * BEEP_TICKS + BEEP_DELAY))
            digitalWrite(VIBE_CTRL_PIN, HIGH);
        else
            digitalWrite(VIBE_CTRL_PIN, LOW);
        if (elapsed > 3 * BEEP_TICKS)
            vibe_action = 0;
    }
    // boop
    else if (vibe_action == 2)
    {
        uint16_t elapsed = (uint16_t)(tick_counter - vibe_start);
        if (elapsed < BOOP_TICKS)
            digitalWrite(VIBE_CTRL_PIN, HIGH);
        else
            digitalWrite(VIBE_CTRL_PIN, LOW);
        if (elapsed > BOOP_TICKS)
            vibe_action = 0;
    }

    if (led_action == 0)
    {
        digitalWrite(LEDA_PIN, HIGH);
        digitalWrite(LEDB_PIN, HIGH);
    }
    else if (led_action == 1)
    {
        digitalWrite(LEDA_PIN, LOW);
        digitalWrite(LEDB_PIN, LOW);
    }
    else if (led_action == 2)
    {
        if (led_counter == 0) led_duty = 256;
        else if ((led_counter % 20) == 0) led_duty = led_duty * 3 / 4;
        led_counter = (led_counter + 1) % 2500;
    }
    else if (led_action == 3 || led_action == 4)
    {
        if (led_counter == 0) led_duty = 256;
        else if ((led_counter % 10) == 0) led_duty = led_duty * 3 / 4;
        led_counter = (led_counter + 1) % 250;
    }

    ++tick_counter;
}

static uint32_t pwm_counter = 0;

ISR(TCB1_INT_vect)
{
    TCB1.INTFLAGS = TCB_CAPT_bm;

    if (led_action > 1)
    {
        uint16_t where = pwm_counter % 256;
        if (where < led_duty)
        {
            if (led_action == 2 || led_action == 3) digitalWrite(LEDA_PIN, LOW);
            if (led_action == 2 || led_action == 4) digitalWrite(LEDB_PIN, LOW);
        }
        else
        {
            if (led_action == 2 || led_action == 3) digitalWrite(LEDA_PIN, HIGH);
            if (led_action == 2 || led_action == 4) digitalWrite(LEDB_PIN, HIGH);
        }
    }

    ++pwm_counter;
}

static void onWireReceive(int byteCount)
{
    while (byteCount > 0)
    {
        int dd = Wire.read();
        if (dd == -1) break;
        if (recvBufPtr == recvBufSz) continue;
        recvBuf[recvBufPtr++] = (uint8_t)dd;
    }
}

static void onWireRequest()
{
    Wire.write((const uint8_t *)sendBuf, sendBytes);
}

void loop()
{
    uint8_t buf[recvBufSz];
    int nBytes;
    cli();
    nBytes = recvBufPtr;
    memcpy(buf, (const void *)recvBuf, nBytes);
    recvBufPtr = 0;
    sei();

    for (int i = 0; i < nBytes; ++i)
    {
        handleCommand(buf[i]);
    }
}

void handleCommand(uint8_t cmd)
{
    if (cmd == 0x00) // get readings
    {
        InputReadings data;
        cli();
        data.tuner = tunerLog.getAvg();
        data.aKnob = aKnobLog.getAvg();
        data.bKnob = bKnobLog.getAvg();
        data.cKnob = cKnobLog.getAvg();
        data.swtch = swtch;
        sendBytes = sizeof(InputReadings) + 1;
        memcpy((void *)sendBuf, &data, sendBytes);
        uint8_t check = 0;
        for (int i = 0; i < sendBytes - 2; ++i)
            check ^= sendBuf[i];
        sendBuf[sendBytes - 1] = check;
        sei();
    }
    else if (cmd == 0x10) // light off
    {
        digitalWrite(LIGHT_CTRL_PIN, LOW);
    }
    else if (cmd == 0x11) // light on
    {
        digitalWrite(LIGHT_CTRL_PIN, HIGH);
    }
    else if (cmd == 0x20) // beep-beep
    {
        cli();
        // Only if currently not vibing
        if (vibe_action == 0)
        {
            vibe_action = 1;
            vibe_start = tick_counter;
        }
        sei();
    }
    else if (cmd == 0x21) // boop
    {
        cli();
        vibe_action = 2;
        vibe_start = tick_counter;
        sei();
    }
    else if (cmd == 0x30 || cmd == 0x31) // LEDs off; both LEDS on
    {
        cli();
        led_action = cmd - 0x30;
        sei();
    }
    else if (cmd == 0x32 || cmd == 0x33 || cmd == 0x34) // Both LEDs pum pum; blink A; blink B
    {
        cli();
        led_action = cmd - 0x30;
        led_counter = 0;
        led_duty = 256;
        pwm_counter = 0;
        sei();
    }
}
