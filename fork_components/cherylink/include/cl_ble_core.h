/*
 * cl_ble_core: the CheryLink Bluetooth channel's protocol logic (value
 * table, frame building, app message parsing, ABRP handover rule). PURE
 * C, host-tested (host_test/test_ble.c); cl_ble.c is the ESP-IDF glue.
 * Wire format: BLE_PROTOCOL.md.
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CLB_PROTO       1
#define CLB_MAX_VALS    96    /* distinct autopid names kept           */
#define CLB_NAME_MAX    32    /* incl. NUL; longer names are skipped   */
#define CLB_LINE_MAX    256   /* one app->device line                  */
#define CLB_HOLD_S      60.0  /* ABRP stays with the phone this long
                                 after its last hello / heartbeat      */

typedef struct
{
    char   name[CLB_NAME_MAX];
    double v;
    bool   dirty;
} clb_val_t;

typedef struct
{
    clb_val_t vals[CLB_MAX_VALS];
    size_t    n;
    uint32_t  skipped;        /* names refused (too long / table full) */

    /* app -> device */
    char     line[CLB_LINE_MAX];
    size_t   line_n;
    bool     line_overflow;
    bool     phone_abrp;      /* the app said it sends to ABRP         */
    double   last_msg;        /* s, 0 = never                          */
    char     app[16];         /* "ios" / "android" from hello          */
    uint32_t msgs;
} clb_t;

void clb_init(clb_t *c);

/** Latest value of an autopid name; marks it for the next frame. */
bool clb_put(clb_t *c, const char *name, double v);

/** Every known value goes out again (a new subscriber). */
void clb_mark_all_dirty(clb_t *c);

/** `{"t":"v","ts":..,"v":{...}}\n` with the changed values that fit;
 *  the rest stay marked for the next call. 0 = nothing changed. */
size_t clb_values_frame(clb_t *c, char *buf, size_t cap, double ts);

/** Bytes the app wrote; complete lines are acted on at @p now. */
void clb_rx(clb_t *c, const uint8_t *d, size_t n, double now);

/** The link dropped: forget a half line (the hold runs out on its own). */
void clb_link_down(clb_t *c);

/** True while the phone owns ABRP (WiCAN's own ABRP sender holds). */
bool clb_abrp_hold(const clb_t *c, double now);

#ifdef __cplusplus
}
#endif
