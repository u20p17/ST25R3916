#pragma once
#include <freertos/FreeRTOS.h>
inline int64_t fakeMicros = 0;
inline TickType_t xTaskGetTickCount() { return fakeMicros / 1000; }
inline void vTaskDelay(TickType_t n) { fakeMicros += int64_t(n) * 1000; }
inline void taskYIELD() { fakeMicros += 10; }
