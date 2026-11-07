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
