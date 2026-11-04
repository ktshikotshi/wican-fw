/*
 * cl_engine: CheryLink drive / charge / efficiency logic, ported from
 * the CheryLink app (ktshikotshi/chery-link: DriveRecorder, EnergyTracker,
 * EfficiencyProfile, ChargeCalibration). PURE C: no ESP-IDF, no I/O, so
 * it builds and runs on the host (host_test/).
 *
 * Input is the autopid sample stream: one named value at a time, with
 * the time it arrived (seconds, epoch when the clock is valid). The
 * Chery Q WiCAN profiles (vehicle_profiles/chery/q.json, q-full.json)
 * supply the names below. Nothing here touches the bus.
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

/* The autopid parameter names the engine listens to. */
typedef enum
{
    CL_SOC = 0,       /* SOC (dash-mapped, %)                    */
    CL_SOC_BMS,       /* SOC_BMS (raw BMS, %)                    */
    CL_SOH,           /* SOH %                                   */
    CL_HV_V,          /* HV_V pack V                             */
    CL_HV_A,          /* HV_A pack A (+ = out)                   */
    CL_HV_W,          /* HV_W pack kW (+ = out)                  */
    CL_SPEED,         /* SPEED km/h                              */
    CL_ODO,           /* ODOMETER km                             */
    CL_PARK,          /* PARK_BRAKE 1 = in P                     */
    CL_GEAR,          /* GEAR 1P 2R 3N 4D                        */
    CL_CHARGING,      /* CHARGING (7E6 4314) 1 = charging        */
    CL_CHARGING_DC,   /* CHARGING_DC (7E5 0002) 1 = DC           */
    CL_EXT_TEMP,      /* TMP_A °C                                */
    CL_BATT_TEMP,     /* BATT_TEMP °C                            */
    CL_AH_COUNTER,    /* AH_CHARGED_COUNTER (FD0A u32@0) Ah      */
    CL_MIN_TO_FULL,   /* MIN_TO_FULL min                         */
    CL_CELL_MAX,      /* CELL_MAX_MV                             */
    CL_CELL_MIN,      /* CELL_MIN_MV                             */
    CL_CELL_AVG,      /* CELL_AVG_V                              */
    CL_TEMP_MAX,      /* BATT_TEMP_MAX °C                        */
    CL_TEMP_MIN,      /* BATT_TEMP_MIN °C                        */
    CL_COOLANT_IN,    /* COOLANT_IN °C                           */
    CL_COOLANT_OUT,   /* COOLANT_OUT °C                          */
    CL_AUX_V,         /* AUX_BATT_V 12 V                         */
    CL_DCDC_V,        /* DCDC_12V                                */
    CL_CYCLES_C,      /* CHARGE_CYCLES                           */
    CL_CYCLES_D,      /* DISCHARGE_CYCLES                        */
    CL_HVAC_KW,       /* HVAC_KW compressor kW                   */
    CL_HEATER_ON,     /* HEATER_ON                               */
    CL_OBC_AC_A,      /* OBC_AC_A                                */
    CL_KEY_COUNT
} cl_key_t;

/* GPS (the u-blox / ESPNetLink add-on, published by main as gps_*). Kept
 * outside val[]/at[] so the v1 state blob stays a prefix of v2. */
#define CL_LAT 100 /* gps_latitude  */
#define CL_LON 101 /* gps_longitude */

/** autopid name -> key, or -1 when the engine ignores it. */
int cl_key_from_name(const char *name);
/** key -> autopid name. */
const char *cl_key_name(int key);

#define CL_PACK_KWH        41.278 /* Gotion LFP 126s x 105 Ah             */
#define CL_RANGE_WINDOW_KM 50     /* range estimate averages the last 50 km */
#define CL_SEED_WH_KM      140.0  /* consumption assumed for km not driven  */
#define CL_BARS            100    /* recent 0.1 km Wh/km bars (10 km)       */
#define CL_SPEED_BANDS     7      /* 0-20 .. 100-120, 120+                  */
#define CL_TEMP_BANDS      4      /* <10, 10-20, 20-30, 30+ °C              */
#define CL_ROUTE_MAX       1000   /* route points kept per drive            */
#define CL_ROUTE_STEP_M    100.0  /* first spacing; doubles when full       */

typedef struct
{
    double start, end;      /* s                                     */
    double km;
    double kwh;             /* net out of the pack (drive - regen)   */
    double regen_kwh;
    double soc_start, soc_end;   /* NAN when unknown                 */
    double odo_start, odo_end;   /* NAN when unknown                 */
    double max_speed;
    double avg_temp;             /* NAN when unknown                 */
    double hvac_kwh;             /* compressor energy during the trip */
} cl_trip_t;

typedef struct
{
    double start, end;      /* s                                     */
    bool   dc;
    double kwh;             /* best estimate into the pack           */
    double kwh_vi;          /* V x I integrated                      */
    double kwh_counter;     /* FD0A counter steps x pack V           */
    double soc_start, soc_end;
    double peak_kw;
    double avg_temp;
    double odo;             /* NAN when unknown                      */
} cl_charge_t;

typedef enum
{
    CL_EVT_TRIP = 1,
    CL_EVT_CHARGE = 2,
} cl_evt_type_t;

typedef void (*cl_event_cb_t)(cl_evt_type_t type, const void *rec, void *ctx);

typedef struct
{
    double km, wh;
} cl_band_t;

/* The whole state. POD with no pointers, so it can be saved and loaded
 * as one blob (cherylink.c checks magic + version + size). */
typedef struct
{
    uint32_t magic;
    uint32_t version;

    /* latest value of every key and when it arrived (0 = never) */
    double val[CL_KEY_COUNT];
    double at[CL_KEY_COUNT];
    bool   seen_hv_w;

    double last_ingest;

    /* power / speed left sums */
    double kw_last, kw_last_t;     /* t = 0: no chain */
    double kmh_last, kmh_last_t;
    double hvac_last, hvac_last_t;

    /* recorder */
    bool        trip_open;
    cl_trip_t   trip;
    double      still_since;   /* 0 = moving */
    double      still_kwh;
    bool        charge_open;
    cl_charge_t charge;
    double      not_charging_since; /* 0 = charging */
    double      ah_last;            /* NAN = no counter reading yet */
    double      temp_sum;
    uint32_t    temp_count;

    /* energy tracker */
    double   seg_km, seg_wh;               /* current 0.1 km bar       */
    float    bars[CL_BARS];                /* ring of finished bars    */
    uint16_t bar_head, bar_count;
    float    range_kms[CL_RANGE_WINDOW_KM];/* ring of finished 1 km    */
    uint16_t range_head, range_count;
    double   range_part_km, range_part_wh;

    /* efficiency bands: 0.5 km stretches by average speed / ext temp */
    double    band_seg_km, band_seg_wh, band_seg_s;
    cl_band_t by_speed[CL_SPEED_BANDS];
    cl_band_t by_temp[CL_TEMP_BANDS];

    /* lifetime totals since the history began */
    double   total_km, total_kwh, total_regen_kwh, total_charged_kwh;
    uint32_t trips, charges;
    double   since;            /* when the history began */

    /* ---- v2 (2026-11-04): location. Appended so a v1 blob loads as a
     * prefix (cherylink.c). NAN = unknown. ---- */
    double   lat, lat_at, lon, lon_at;        /* last fix and when      */
    double   trip_start_lat, trip_start_lon;
    double   trip_end_lat, trip_end_lon;
    double   charge_lat, charge_lon;
    double   route_step_m;
    uint32_t route_n;
    float    route[CL_ROUTE_MAX][2];          /* lat, lon of open drive */
} cl_engine_t;

/** Size of a v1 state blob (everything before the v2 fields). */
#define CL_V1_SIZE offsetof(cl_engine_t, lat)
/** Sets the v2 fields of an engine loaded from a v1 blob. */
void cl_engine_init_v2(cl_engine_t *e);

#define CL_MAGIC   0x434C4E4BU /* "CLNK" */
#define CL_VERSION 2U

void cl_engine_init(cl_engine_t *e);

/** One sample. Returns true when a trip or charge ended (cb called). */
bool cl_ingest(cl_engine_t *e, int key, double value, double t,
               cl_event_cb_t cb, void *ctx);

/** Housekeeping with no sample (call every few seconds): closes an
 *  open trip/charge once data stopped for CL_GAP_END_S. */
bool cl_tick(cl_engine_t *e, double t, cl_event_cb_t cb, void *ctx);

/** Adapter going to sleep / data source gone: close a trip; a charge
 *  stays open when @p keep_charge (it carries on after the wake). */
bool cl_flush(cl_engine_t *e, double t, bool keep_charge,
              cl_event_cb_t cb, void *ctx);

/** Wh/km over the last 50 km (km not driven yet count at 140). */
double cl_range_wh_km(const cl_engine_t *e);
/** km left from the current SOC at cl_range_wh_km(), NAN without SOC. */
double cl_est_range_km(const cl_engine_t *e);
/** Wh/km right now (NAN below 5 km/h). */
double cl_instant_wh_km(const cl_engine_t *e);
/** Average over the last @p km of bars, NAN before the first bar. */
double cl_avg_wh_km(const cl_engine_t *e, double km);
/** Recent bars oldest first into @p out (max CL_BARS), returns count. */
size_t cl_bars(const cl_engine_t *e, float *out, size_t max);

/** Value of @p key if it arrived within @p max_age_s of @p t, else NAN. */
double cl_fresh(const cl_engine_t *e, int key, double t, double max_age_s);

/* Tunables (DriveRecorder defaults; max gap wider: WiCAN polls each PID
 * every few seconds, not every 0.5 s like the app). */
#define CL_PARKED_END_S   180.0
#define CL_CHARGE_END_S   120.0
#define CL_MIN_TRIP_KM    0.2
#define CL_MIN_CHARGE_KWH 0.05
#define CL_MAX_GAP_S      30.0
#define CL_GAP_END_S      600.0
#define CL_CHARGE_GAP_S   (12.0 * 3600.0)

#ifdef __cplusplus
}
#endif
