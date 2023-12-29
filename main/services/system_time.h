#ifndef SYSTEM_TIME_H_INCLUDED
#define SYSTEM_TIME_H_INCLUDED

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"


#define time_after_or_equal(a, b) (((long)((b) - (a)) <= 0))

static inline __attribute__((always_inline)) uint8_t is_expired(unsigned long start, unsigned long current,
                                                                unsigned long delay) {
    return time_after_or_equal(current, start + delay);
}

#define get_millis() (xTaskGetTickCount() * portTICK_PERIOD_MS)

#endif
