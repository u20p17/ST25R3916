#pragma once
#include <esp_err.h>
inline const char* esp_err_to_name(esp_err_t) { return "test error"; }
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGD(...) ((void)0)
#define ESP_LOGV(...) ((void)0)
