#define ESP_LOGI(tag, fmt, ...) printf("[%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW ESP_LOGI
#define ESP_LOGD ESP_LOGI
#define ESP_LOGE ESP_LOGI