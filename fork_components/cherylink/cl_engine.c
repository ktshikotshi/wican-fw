/*
 * cl_engine.c: CheryLink drive / charge / efficiency logic (pure C).
 * Port of the CheryLink app's DriveRecorder + EnergyTracker +
 * EfficiencyProfile (speed bands) + ChargeCalibration (counter steps x
 * pack V). Differences from the app, because WiCAN delivers one value
 * at a time (each PID on its own schedule) instead of one round:
 *  - distance is a left sum over SPEED samples, energy a left sum over
 *    HV_W samples (HV_V x HV_A when the profile has no HV_W);
 *  - the max gap still integrated is 30 s (the app polls ~0.5 s).
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include "cl_engine.h"

#include <math.h>
#include <string.h>

static const char *const NAMES[CL_KEY_COUNT] =
{
    [CL_SOC] = "SOC",               [CL_SOC_BMS] = "SOC_BMS",
    [CL_SOH] = "SOH",               [CL_HV_V] = "HV_V",
    [CL_HV_A] = "HV_A",             [CL_HV_W] = "HV_W",
    [CL_SPEED] = "SPEED",           [CL_ODO] = "ODOMETER",
    [CL_PARK] = "PARK_BRAKE",       [CL_GEAR] = "GEAR",
    [CL_CHARGING] = "CHARGING",     [CL_CHARGING_DC] = "CHARGING_DC",
    [CL_EXT_TEMP] = "TMP_A",        [CL_BATT_TEMP] = "BATT_TEMP",
    [CL_AH_COUNTER] = "AH_CHARGED_COUNTER",
    [CL_MIN_TO_FULL] = "MIN_TO_FULL",
    [CL_CELL_MAX] = "CELL_MAX_MV",  [CL_CELL_MIN] = "CELL_MIN_MV",
    [CL_CELL_AVG] = "CELL_AVG_V",
    [CL_TEMP_MAX] = "BATT_TEMP_MAX", [CL_TEMP_MIN] = "BATT_TEMP_MIN",
    [CL_COOLANT_IN] = "COOLANT_IN", [CL_COOLANT_OUT] = "COOLANT_OUT",
    [CL_AUX_V] = "AUX_BATT_V",      [CL_DCDC_V] = "DCDC_12V",
    [CL_CYCLES_C] = "CHARGE_CYCLES", [CL_CYCLES_D] = "DISCHARGE_CYCLES",
    [CL_HVAC_KW] = "HVAC_KW",       [CL_HEATER_ON] = "HEATER_ON",
    [CL_OBC_AC_A] = "OBC_AC_A",
};

int cl_key_from_name(const char *name)
{
    if (name == NULL)
    {
        return -1;
    }
    for (int i = 0; i < CL_KEY_COUNT; i++)
    {
        if (NAMES[i] != NULL && strcmp(NAMES[i], name) == 0)
        {
            return i;
        }
    }
    return -1;
}

const char *cl_key_name(int key)
{
    return (key >= 0 && key < CL_KEY_COUNT) ? NAMES[key] : "";
}

void cl_engine_init(cl_engine_t *e)
{
    memset(e, 0, sizeof(*e));
    e->magic = CL_MAGIC;
    e->version = CL_VERSION;
    e->ah_last = NAN;
}

static bool has(const cl_engine_t *e, int k)
{
    return e->at[k] > 0;
}

static double val_or_nan(const cl_engine_t *e, int k)
{
    return has(e, k) ? e->val[k] : NAN;
}

double cl_fresh(const cl_engine_t *e, int key, double t, double max_age_s)
{
    if (key < 0 || key >= CL_KEY_COUNT || !has(e, key) ||
        t - e->at[key] > max_age_s)
    {
        return NAN;
    }
    return e->val[key];
}

/* ---- energy tracker ---------------------------------------------------- */

static void push_bar(cl_engine_t *e, float v)
{
    e->bars[e->bar_head] = v;
    e->bar_head = (uint16_t)((e->bar_head + 1) % CL_BARS);
    if (e->bar_count < CL_BARS)
    {
        e->bar_count++;
    }
}

static void push_range(cl_engine_t *e, float v)
{
    e->range_kms[e->range_head] = v;
    e->range_head = (uint16_t)((e->range_head + 1) % CL_RANGE_WINDOW_KM);
    if (e->range_count < CL_RANGE_WINDOW_KM)
    {
        e->range_count++;
    }
}

static int temp_band(double c)
{
    if (isnan(c))
    {
        return -1;
    }
    if (c < 10)
    {
        return 0;
    }
    if (c < 20)
    {
        return 1;
    }
    return c < 30 ? 2 : 3;
}

/* dkm / dwh of driving (never charging), ds = seconds of the speed chain */
static void track(cl_engine_t *e, double dkm, double dwh, double ds, double t)
{
    e->seg_km += dkm;
    e->seg_wh += dwh;
    if (e->seg_km >= 0.1)
    {
        push_bar(e, (float)(e->seg_wh / e->seg_km));
        e->seg_km = 0;
        e->seg_wh = 0;
    }

    e->range_part_km += dkm;
    e->range_part_wh += dwh;
    if (e->range_part_km >= 1.0)
    {
        push_range(e, (float)(e->range_part_wh / e->range_part_km));
        e->range_part_km = 0;
        e->range_part_wh = 0;
    }

    e->band_seg_km += dkm;
    e->band_seg_wh += dwh;
    e->band_seg_s += ds;
    if (e->band_seg_km >= 0.5 && e->band_seg_s > 0)
    {
        double kmh = e->band_seg_km / (e->band_seg_s / 3600.0);
        int si = (int)(kmh / 20.0);
        if (si >= CL_SPEED_BANDS)
        {
            si = CL_SPEED_BANDS - 1;
        }
        if (si < 0)
        {
            si = 0;
        }
        e->by_speed[si].km += e->band_seg_km;
        e->by_speed[si].wh += e->band_seg_wh;
        int ti = temp_band(cl_fresh(e, CL_EXT_TEMP, t, 900));
        if (ti >= 0)
        {
            e->by_temp[ti].km += e->band_seg_km;
            e->by_temp[ti].wh += e->band_seg_wh;
        }
        e->band_seg_km = 0;
        e->band_seg_wh = 0;
        e->band_seg_s = 0;
    }
}

static void track_reset_partial(cl_engine_t *e)
{
    e->seg_km = 0;
    e->seg_wh = 0;
    e->band_seg_km = 0;
    e->band_seg_wh = 0;
    e->band_seg_s = 0;
}

double cl_range_wh_km(const cl_engine_t *e)
{
    double wh = e->range_part_wh, km = e->range_part_km;
    for (int i = 0; i < e->range_count; i++)
    {
        wh += e->range_kms[i];
    }
    km += e->range_count;
    double w = CL_RANGE_WINDOW_KM;
    double avg = km >= w ? wh / km : (wh + CL_SEED_WH_KM * (w - km)) / w;
    return avg < 60 ? 60 : avg;
}

double cl_est_range_km(const cl_engine_t *e)
{
    if (!has(e, CL_SOC))
    {
        return NAN;
    }
    double soc = e->val[CL_SOC];
    if (soc < 0)
    {
        soc = 0;
    }
    return round(soc / 100.0 * CL_PACK_KWH * 1000.0 / cl_range_wh_km(e));
}

double cl_instant_wh_km(const cl_engine_t *e)
{
    if (!has(e, CL_SPEED) || e->kw_last_t <= 0 || e->val[CL_SPEED] < 5)
    {
        return NAN;
    }
    return e->kw_last / e->val[CL_SPEED] * 1000.0;
}

size_t cl_bars(const cl_engine_t *e, float *out, size_t max)
{
    size_t n = e->bar_count < max ? e->bar_count : max;
    size_t first = (e->bar_head + CL_BARS - n) % CL_BARS;
    for (size_t i = 0; i < n; i++)
    {
        out[i] = e->bars[(first + i) % CL_BARS];
    }
    return n;
}

double cl_avg_wh_km(const cl_engine_t *e, double km)
{
    if (e->bar_count == 0)
    {
        return NAN;
    }
    int n = (int)lround(km / 0.1);
    if (n < 1)
    {
        n = 1;
    }
    if (n > e->bar_count)
    {
        n = e->bar_count;
    }
    float tmp[CL_BARS];
    size_t got = cl_bars(e, tmp, CL_BARS);
    double s = 0;
    for (size_t i = got - (size_t)n; i < got; i++)
    {
        s += tmp[i];
    }
    return s / n;
}

/* ---- recorder ------------------------------------------------------------ */

static bool close_trip(cl_engine_t *e, double end, cl_event_cb_t cb, void *ctx)
{
    if (!e->trip_open)
    {
        return false;
    }
    cl_trip_t tr = e->trip;
    e->trip_open = false;
    e->still_since = 0;
    e->still_kwh = 0;
    if (end < tr.end)
    {
        tr.end = end;
    }
    if (tr.end < tr.start)
    {
        tr.end = tr.start;
    }
    if (!isnan(tr.odo_start) && !isnan(tr.odo_end))
    {
        double d = tr.odo_end - tr.odo_start;
        if (d > 0.05 && d < tr.km * 1.5 + 2)
        {
            tr.km = d;
        }
    }
    tr.avg_temp = e->temp_count ? e->temp_sum / e->temp_count : NAN;
    if (tr.km < CL_MIN_TRIP_KM)
    {
        return false;
    }
    e->total_km += tr.km;
    e->total_kwh += tr.kwh;
    e->total_regen_kwh += tr.regen_kwh;
    e->trips++;
    if (cb)
    {
        cb(CL_EVT_TRIP, &tr, ctx);
    }
    return true;
}

static bool close_charge(cl_engine_t *e, double end, cl_event_cb_t cb,
                         void *ctx)
{
    if (!e->charge_open)
    {
        return false;
    }
    cl_charge_t c = e->charge;
    e->charge_open = false;
    e->not_charging_since = 0;
    if (end < c.end)
    {
        c.end = end;
    }
    if (c.end < c.start)
    {
        c.end = c.start;
    }
    if (c.kwh_counter >= CL_MIN_CHARGE_KWH)
    {
        c.kwh = c.kwh_counter;
    }
    else
    {
        c.kwh = c.kwh_vi;
    }
    if (c.kwh < CL_MIN_CHARGE_KWH && !isnan(c.soc_start) &&
        !isnan(c.soc_end) && c.soc_end > c.soc_start)
    {
        c.kwh = (c.soc_end - c.soc_start) / 100.0 * CL_PACK_KWH;
    }
    c.avg_temp = e->temp_count ? e->temp_sum / e->temp_count : NAN;
    if (c.kwh < CL_MIN_CHARGE_KWH && (c.end - c.start) < 300)
    {
        return false;
    }
    e->total_charged_kwh += c.kwh;
    e->charges++;
    if (cb)
    {
        cb(CL_EVT_CHARGE, &c, ctx);
    }
    return true;
}

static void break_chains(cl_engine_t *e)
{
    e->kw_last_t = 0;
    e->kmh_last_t = 0;
    e->hvac_last_t = 0;
}

bool cl_flush(cl_engine_t *e, double t, bool keep_charge, cl_event_cb_t cb,
              void *ctx)
{
    bool ended = false;
    double tend = e->still_since > 0 ? e->still_since
                                     : (e->trip_open ? e->trip.end : t);
    ended |= close_trip(e, tend, cb, ctx);
    if (!keep_charge || e->not_charging_since > 0)
    {
        double cend = e->not_charging_since > 0
                          ? e->not_charging_since
                          : (e->charge_open ? e->charge.end : t);
        ended |= close_charge(e, cend, cb, ctx);
    }
    break_chains(e);
    track_reset_partial(e);
    return ended;
}

/* data stopped for longer than CL_GAP_END_S (car off, adapter asleep) */
static bool gap_end(cl_engine_t *e, double t, cl_event_cb_t cb, void *ctx)
{
    double last = e->last_ingest;
    bool ended = false;
    if (e->charge_open && t - last <= CL_CHARGE_GAP_S)
    {
        /* The charge may still be running: end only the trip and wait
         * for a fresh CHARGING read. */
        double tend = e->still_since > 0 ? e->still_since
                                         : (e->trip_open ? e->trip.end : last);
        ended |= close_trip(e, tend, cb, ctx);
        break_chains(e);
        e->at[CL_CHARGING] = 0;
        e->at[CL_CHARGING_DC] = 0;
        e->not_charging_since = 0;
    }
    else
    {
        ended |= cl_flush(e, last, false, cb, ctx);
    }
    return ended;
}

bool cl_tick(cl_engine_t *e, double t, cl_event_cb_t cb, void *ctx)
{
    if (e->last_ingest > 0 && t - e->last_ingest > CL_GAP_END_S &&
        (e->trip_open || e->kw_last_t > 0 || e->kmh_last_t > 0))
    {
        return gap_end(e, t, cb, ctx);
    }
    return false;
}

bool cl_ingest(cl_engine_t *e, int key, double v, double t, cl_event_cb_t cb,
               void *ctx)
{
    if (key < 0 || key >= CL_KEY_COUNT || isnan(v) || t <= 0)
    {
        return false;
    }
    bool ended = false;
    if (e->last_ingest > 0 && t - e->last_ingest > CL_GAP_END_S)
    {
        ended |= gap_end(e, t, cb, ctx);
    }
    e->last_ingest = t;
    if (e->since <= 0)
    {
        e->since = t;
    }

    double soc_before = val_or_nan(e, CL_SOC);
    double odo_before = val_or_nan(e, CL_ODO);
    e->val[key] = v;
    e->at[key] = t;
    if (key == CL_HV_W)
    {
        e->seen_hv_w = true;
    }

    bool charging = (has(e, CL_CHARGING) && e->val[CL_CHARGING] >= 0.5) ||
                    (has(e, CL_CHARGING_DC) && e->val[CL_CHARGING_DC] >= 0.5);
    double speed = has(e, CL_SPEED) ? e->val[CL_SPEED] : 0;
    bool moving = speed > 3;
    bool in_gear = has(e, CL_PARK) ? e->val[CL_PARK] < 0.5 : moving;

    /* ---- integrate (left sums) ---- */
    double dkm = 0, ds = 0, dkwh = 0, ddrive = 0, dhvac = 0;
    bool power_sample = false;
    double kw = 0;
    if (key == CL_SPEED)
    {
        if (e->kmh_last_t > 0)
        {
            double dt = t - e->kmh_last_t;
            if (dt > 0 && dt <= CL_MAX_GAP_S)
            {
                dkm = (e->kmh_last > 0 ? e->kmh_last : 0) * dt / 3600.0;
                ds = dt;
            }
        }
        e->kmh_last = v;
        e->kmh_last_t = t;
        if (e->trip_open && v > e->trip.max_speed && !charging)
        {
            e->trip.max_speed = v;
        }
    }
    if (key == CL_HV_W)
    {
        power_sample = true;
        kw = v;
    }
    else if (key == CL_HV_A && !e->seen_hv_w &&
             !isnan(cl_fresh(e, CL_HV_V, t, 15)))
    {
        power_sample = true;
        kw = e->val[CL_HV_V] * v / 1000.0;
    }
    if (power_sample)
    {
        if (e->kw_last_t > 0)
        {
            double dt = t - e->kw_last_t;
            if (dt > 0 && dt <= CL_MAX_GAP_S)
            {
                dkwh = e->kw_last * dt / 3600.0;
                /* energy flowing in while standing is a charger, not regen */
                ddrive = (speed < 1 && e->kw_last < 0) ? 0 : dkwh;
            }
        }
        e->kw_last = kw;
        e->kw_last_t = t;
    }
    if (key == CL_HVAC_KW)
    {
        if (e->hvac_last_t > 0)
        {
            double dt = t - e->hvac_last_t;
            if (dt > 0 && dt <= CL_MAX_GAP_S)
            {
                dhvac = e->hvac_last * dt / 3600.0;
            }
        }
        e->hvac_last = v;
        e->hvac_last_t = t;
    }

    /* ---- charging ---- */
    if (charging)
    {
        e->not_charging_since = 0;
        ended |= close_trip(e, t, cb, ctx);
        if (!e->charge_open)
        {
            e->charge_open = true;
            memset(&e->charge, 0, sizeof(e->charge));
            e->charge.start = t;
            e->charge.end = t;
            e->charge.soc_start = !isnan(soc_before) ? soc_before
                                                     : val_or_nan(e, CL_SOC);
            e->charge.soc_end = NAN;
            e->charge.avg_temp = NAN;
            e->charge.odo = val_or_nan(e, CL_ODO);
            e->ah_last = NAN;
            e->temp_sum = 0;
            e->temp_count = 0;
            track_reset_partial(e);
        }
    }
    else if (e->charge_open)
    {
        if (e->not_charging_since <= 0)
        {
            e->not_charging_since = t;
        }
        if (t - e->not_charging_since >= CL_CHARGE_END_S || moving)
        {
            ended |= close_charge(e, e->not_charging_since, cb, ctx);
        }
    }
    if (e->charge_open)
    {
        cl_charge_t *c = &e->charge;
        if (has(e, CL_CHARGING_DC) && e->val[CL_CHARGING_DC] >= 0.5)
        {
            c->dc = true;
        }
        if (power_sample && -kw > c->peak_kw)
        {
            c->peak_kw = -kw;
        }
        if (dkwh < 0)
        {
            c->kwh_vi += -dkwh;
        }
        if (key == CL_AH_COUNTER)
        {
            if (!isnan(e->ah_last) && v > e->ah_last && v - e->ah_last < 60)
            {
                double volts = cl_fresh(e, CL_HV_V, t, 60);
                if (isnan(volts))
                {
                    volts = 400;
                }
                c->kwh_counter += (v - e->ah_last) * volts / 1000.0;
            }
            e->ah_last = v;
        }
        if (charging)
        {
            c->end = t;
            c->soc_end = val_or_nan(e, CL_SOC);
        }
    }
    else if (key == CL_AH_COUNTER)
    {
        e->ah_last = v;
    }

    /* ---- driving ---- */
    if (!charging && !e->charge_open)
    {
        if (!e->trip_open && (moving || in_gear))
        {
            e->trip_open = true;
            memset(&e->trip, 0, sizeof(e->trip));
            e->trip.start = t;
            e->trip.end = t;
            e->trip.soc_start = !isnan(soc_before) ? soc_before
                                                   : val_or_nan(e, CL_SOC);
            e->trip.soc_end = NAN;
            e->trip.odo_start = !isnan(odo_before) ? odo_before
                                                   : val_or_nan(e, CL_ODO);
            e->trip.odo_end = NAN;
            e->trip.avg_temp = NAN;
            e->trip.max_speed = speed;
            e->still_since = 0;
            e->still_kwh = 0;
            e->temp_sum = 0;
            e->temp_count = 0;
        }
        if (e->trip_open)
        {
            cl_trip_t *tr = &e->trip;
            if (isnan(tr->soc_start))
            {
                tr->soc_start = val_or_nan(e, CL_SOC);
            }
            if (isnan(tr->odo_start))
            {
                tr->odo_start = val_or_nan(e, CL_ODO);
            }
            tr->km += dkm;
            tr->hvac_kwh += dhvac;
            if (ddrive < 0)
            {
                tr->regen_kwh += -ddrive;
            }
            if (moving || in_gear)
            {
                /* energy used standing counts only if the drive carries on */
                tr->kwh += ddrive + e->still_kwh;
                e->still_kwh = 0;
                e->still_since = 0;
                tr->end = t;
                tr->soc_end = val_or_nan(e, CL_SOC);
                tr->odo_end = val_or_nan(e, CL_ODO);
            }
            else
            {
                e->still_kwh += ddrive;
                if (e->still_since <= 0)
                {
                    e->still_since = t;
                }
            }
            if (e->still_since > 0 &&
                t - e->still_since >= CL_PARKED_END_S)
            {
                ended |= close_trip(e, e->still_since, cb, ctx);
            }
        }
        track(e, dkm, ddrive * 1000.0, ds, t);
    }

    if (key == CL_EXT_TEMP && (e->trip_open || e->charge_open))
    {
        e->temp_sum += v;
        e->temp_count++;
    }
    return ended;
}
