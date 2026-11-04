#pragma once
#include <freertos/task.h>
inline int64_t esp_timer_get_time() { return fakeMicros; }
