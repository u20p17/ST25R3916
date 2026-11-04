#pragma once
#include <freertos/task.h>
inline void esp_rom_delay_us(unsigned n) { fakeMicros += n; }
