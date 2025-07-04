#pragma once

#include <stdint.h>
#include <driver/gpio.h>
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"

class LedcStepper
{
    friend bool IRAM_ATTR pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);
    friend void stop_task(void *pvParameter);

public:
    LedcStepper(uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin, bool invert_dir_pin = false, UBaseType_t task_priority = 1, UBaseType_t task_stop_priority = 1);
    void step(int32_t position, uint32_t speed, bool wait = true, bool reset = false);
    void go_to(int32_t position, uint32_t speed, bool wait = true);
    void step_speed(int32_t speed);
    void stop(bool stop_waiting = true, bool from_ISR = false);
    void disable();
    void enable();
    int get_position();
    void reset_position();
    void wait();

private:
    static uint8_t _next_timer_channel;
    static SemaphoreHandle_t _pcnt_mutex;
    TaskHandle_t _task_to_notify_handle = NULL;
    TaskHandle_t _stop_task_handle;
    TaskHandle_t _wait_task_handle = NULL;
    bool _waiting = false;
    uint8_t _step_pin;
    gpio_num_t _dir_pin;
    bool _invert_dir_pin;
    gpio_num_t _en_pin;
    uint32_t _pos{0};
    ledc_timer_t _timer;
    ledc_channel_t _channel;
    bool _enabled = false;
    bool _running = false;
    int _position = 0;
    int _pcnt_watch_point = 0;
    int _pcnt_objective = 0;
    int32_t _last_speed = 0;

    pcnt_unit_handle_t _pcnt_unit = NULL;
    pcnt_channel_handle_t _pcnt_chan = NULL;

    UBaseType_t _task_priority;
    UBaseType_t _task_stop_priority;
};
