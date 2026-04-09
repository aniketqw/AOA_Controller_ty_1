#include "thresholds.h"
#include <string.h>
#include "esp_log.h"
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "THRESHOLDS";

static threshold_entry_t default_table[] = {
    {"Aircraft_A", "TAKEOFF", 0.0f, 18.0f},
    {"Aircraft_A", "CLIMB", 5.0f, 20.0f},
    {"Aircraft_A", "CRUISE", 3.0f, 22.0f},
    {"Aircraft_A", "LANDING", 2.0f, 20.0f},
    {"Aircraft_B", "TAKEOFF", 2.0f, 16.0f},
    {"Aircraft_B", "CLIMB", 4.0f, 18.0f},
    {"Aircraft_B", "CRUISE", 2.0f, 20.0f},
    {"Aircraft_B", "LANDING", 1.0f, 18.0f},
};

static threshold_entry_t *thresholds_table = NULL;
static uint32_t num_threshold_entries = 0;
static bool thresholds_loaded = false;

static void trim(char *s) {
    if (!s) return;
    char *p = s; while (*p && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) p++;
    if (p != s) memmove(s, p, strlen(p) + 1);
    size_t len = strlen(s);
    while (len > 0 && (s[len-1] == ' ' || s[len-1] == '\t' || s[len-1] == '\r' || s[len-1] == '\n')) { s[len-1] = '\0'; len--; }
}

static void load_thresholds_from_csv(void) {
    if (thresholds_loaded) return;
#ifdef THRESHOLDS_CSV_PATH
    const char *csv_path = THRESHOLDS_CSV_PATH;
    FILE *f = fopen(csv_path, "r");
    if (!f) { ESP_LOGW(TAG, "Could not open thresholds CSV at %s; using defaults", csv_path); }
    else {
        char line[256]; uint32_t count = 0;
        while (fgets(line, sizeof(line), f)) { trim(line); if (line[0] == '\0' || line[0] == '#') continue; count++; }
        if (count == 0) { fclose(f); }
        else {
            rewind(f);
            thresholds_table = malloc(sizeof(threshold_entry_t) * count);
            if (!thresholds_table) { ESP_LOGW(TAG, "Malloc failed; using defaults"); fclose(f); }
            else {
                uint32_t idx = 0; while (fgets(line, sizeof(line), f)) {
                    trim(line); if (line[0] == '\0' || line[0] == '#') continue;
                    char *tok = strtok(line, ","); if (!tok) continue; strncpy(thresholds_table[idx].aircraft_type, tok, sizeof(thresholds_table[idx].aircraft_type)-1);
                    tok = strtok(NULL, ","); if (!tok) continue; strncpy(thresholds_table[idx].flight_mode, tok, sizeof(thresholds_table[idx].flight_mode)-1);
                    tok = strtok(NULL, ","); if (!tok) continue; thresholds_table[idx].aoa_low = strtof(tok, NULL);
                    tok = strtok(NULL, ","); if (!tok) continue; thresholds_table[idx].aoa_high = strtof(tok, NULL);
                    trim(thresholds_table[idx].aircraft_type); trim(thresholds_table[idx].flight_mode); idx++;
                }
                num_threshold_entries = idx; fclose(f); thresholds_loaded = true; ESP_LOGI(TAG, "Loaded %u thresholds", num_threshold_entries); return;
            }
        }
    }
#else
    ESP_LOGW(TAG, "THRESHOLDS_CSV_PATH not defined; using defaults");
#endif
    thresholds_table = default_table;
    num_threshold_entries = sizeof(default_table) / sizeof(default_table[0]);
    thresholds_loaded = true;
}

bool thresholds_lookup(const char *aircraft_type, const char *flight_mode, float *aoa_low, float *aoa_high) {
    if (!aircraft_type || !flight_mode || !aoa_low || !aoa_high) return false;
    if (!thresholds_loaded) load_thresholds_from_csv();
    for (uint32_t i = 0; i < num_threshold_entries; i++) {
        if (strcmp(thresholds_table[i].aircraft_type, aircraft_type) == 0 && strcmp(thresholds_table[i].flight_mode, flight_mode) == 0) {
            *aoa_low = thresholds_table[i].aoa_low; *aoa_high = thresholds_table[i].aoa_high; return true;
        }
    }
    ESP_LOGW(TAG, "thresholds_lookup: no entry for %s/%s", aircraft_type, flight_mode);
    return false;
}
