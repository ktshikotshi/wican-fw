/*
 * cherylink internals shared by cherylink.c and cl_ble.c (not public).
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

/** Every autopid value (any name), from the sink: poller context, never
 *  blocks for long. */
void cl_ble_on_value(const char *name, double value);

/** A finished drive (charge=false) or charge as its JSON history line. */
void cl_ble_on_record(bool charge, const char *line);

/** Compact live summary (range, efficiency, open trip/charge) as one JSON
 *  object into buf; returns its length (0 when it did not fit). */
size_t cl_status_compact(char *buf, size_t cap);

/** Opens the Bluetooth pairing window for @p seconds (a new phone can
 *  pair); it closes again on its own unless it was already open (the
 *  ble pairing_at_boot setting). Returns false when Bluetooth is off. */
bool cl_ble_pair_open(int seconds);

typedef struct
{
    bool enabled;      /* Bluetooth on in settings          */
    bool connected;    /* a phone is connected + paired     */
    bool pairing;      /* new phones may pair now           */
    int  pairing_left; /* s until the window closes, 0 = stays open / closed */
    bool phone_abrp;   /* the phone owns ABRP now           */
    char app[16];      /* "ios" / "android" from its hello  */
} cl_ble_info_t;

void cl_ble_info(cl_ble_info_t *out);
