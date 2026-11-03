/*
 * cherylink: CheryLink drive history, charge history, efficiency and
 * battery info on the WiCAN Pro (fork component, Chery Q profiles).
 *
 * Listens to autopid samples only (main chains the value sink): it never
 * sends anything on the bus. Records go to /data/cherylink/ (internal
 * LittleFS) through flash_jobs; the page is /cherylink, the API
 * /api/cherylink, /api/cherylink/drives, /api/cherylink/charges.
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Queue + mutex (main_boot_init, before HTTP registration). */
esp_err_t cherylink_init(void);
/** Routes; must run before web_ui_v2_register (its catch-all). */
esp_err_t cherylink_register_http(void);
/** Loads the saved state and starts the task (after the filesystem). */
esp_err_t cherylink_start(void);
/** Sleep/shutdown: closes a trip (a charge stays open) and saves. */
esp_err_t cherylink_stop(void);

/** autopid value sink (poller context: queue push only, never blocks). */
void cherylink_autopid_sink(const char *name, const char *unit,
                            double value, bool changed);

#ifdef __cplusplus
}
#endif
