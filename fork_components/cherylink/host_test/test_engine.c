/* Host test for cl_engine: cc -I../include ../cl_engine.c test_engine.c -lm */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "cl_engine.h"

static int trips, charges;
static cl_trip_t last_trip;
static cl_charge_t last_charge;

static void cb(cl_evt_type_t t, const void *r, void *ctx)
{
    (void)ctx;
    if (t == CL_EVT_TRIP) { trips++; last_trip = *(const cl_trip_t *)r; }
    else { charges++; last_charge = *(const cl_charge_t *)r; }
}

static void put(cl_engine_t *e, const char *n, double v, double t)
{
    cl_ingest(e, cl_key_from_name(n), v, t, cb, NULL);
}

#define NEAR(a, b, tol) (fabs((a) - (b)) <= (tol))

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    static cl_engine_t e;
    cl_engine_init(&e);
    double t = 1.8e9;

    /* parked, idle */
    put(&e, "SOC", 80, t); put(&e, "ODOMETER", 1000, t);
    put(&e, "PARK_BRAKE", 1, t); put(&e, "CHARGING", 0, t);
    put(&e, "SPEED", 0, t); put(&e, "HV_W", 0.3, t);
    assert(!e.trip_open);

    /* drive 600 s at 50 km/h, 10 kW, samples every 2 s, values staggered */
    put(&e, "PARK_BRAKE", 0, t += 1);
    assert(e.trip_open);
    for (int i = 0; i < 300; i++)
    {
        t += 1; put(&e, "SPEED", 50, t);
        t += 1; put(&e, "HV_W", 10, t);
        if (i % 10 == 0) put(&e, "TMP_A", 22, t);
    }
    put(&e, "SOC", 76, t);
    put(&e, "ODOMETER", 1008.3, t);
    /* stop and park */
    put(&e, "SPEED", 0, t += 1);
    put(&e, "PARK_BRAKE", 1, t += 1);
    for (int i = 0; i < 100; i++) { t += 2; put(&e, "HV_W", 0.3, t); }
    assert(trips == 1);
    printf("trip km %.2f kwh %.3f wh/km %.0f soc %.0f->%.0f temp %.1f\n",
           last_trip.km, last_trip.kwh, last_trip.kwh * 1000 / last_trip.km,
           last_trip.soc_start, last_trip.soc_end, last_trip.avg_temp);
    assert(NEAR(last_trip.km, 8.3, 0.01)); /* odometer wins */
    assert(NEAR(last_trip.kwh, 1.667, 0.05));
    assert(last_trip.soc_start == 80 && last_trip.soc_end == 76);
    printf("bars %u range wh/km %.1f est %.0f km, band 40-60 km %.2f wh/km %.0f\n",
           e.bar_count, cl_range_wh_km(&e), cl_est_range_km(&e),
           e.by_speed[2].km, e.by_speed[2].wh / e.by_speed[2].km);
    printf("bars=%u\n", e.bar_count); assert(e.bar_count >= 70 && e.bar_count <= 84);
    assert(NEAR(e.by_speed[2].wh / e.by_speed[2].km, 200, 2));
    assert(NEAR(e.by_temp[2].km, 8.0, 0.6));

    /* AC charge: 1 h at -6.6 kW, counter +16 Ah at 410 V */
    put(&e, "CHARGING", 1, t += 10);
    assert(e.charge_open);
    put(&e, "AH_CHARGED_COUNTER", 100, t);
    for (int i = 0; i < 1200; i++)
    {
        t += 3;
        put(&e, "HV_W", -6.6, t);
        put(&e, "HV_V", 410, t);
        if (i % 75 == 74) put(&e, "AH_CHARGED_COUNTER", 100 + (i + 1) / 75.0, t);
        put(&e, "CHARGING", 1, t);
    }
    put(&e, "SOC", 92, t);
    put(&e, "CHARGING", 1, t += 1);
    for (int i = 0; i < 50; i++) { t += 3; put(&e, "CHARGING", 0, t); }
    assert(charges == 1);
    printf("charge kwh %.2f vi %.2f counter %.2f peak %.1f soc %.0f->%.0f dc %d min %.0f\n",
           last_charge.kwh, last_charge.kwh_vi, last_charge.kwh_counter,
           last_charge.peak_kw, last_charge.soc_start, last_charge.soc_end,
           last_charge.dc, (last_charge.end - last_charge.start) / 60);
    assert(NEAR(last_charge.kwh_counter, 16 * 0.41, 0.05));
    assert(NEAR(last_charge.kwh_vi, 6.6, 0.1));
    assert(last_charge.soc_start == 76 && last_charge.soc_end == 92);

    /* gap: drive, then data stops -> tick closes the trip */
    put(&e, "PARK_BRAKE", 0, t += 5);
    for (int i = 0; i < 100; i++) { t += 1; put(&e, "SPEED", 30, t); t += 1; put(&e, "HV_W", 5, t); }
    assert(e.trip_open);
    assert(!cl_tick(&e, t + 100, cb, NULL));
    assert(cl_tick(&e, t + 700, cb, NULL));
    assert(trips == 2 && !e.trip_open);
    printf("gap trip km %.2f kwh %.3f\n", last_trip.km, last_trip.kwh);

    /* charge open across sleep: flush(keep) keeps it */
    put(&e, "CHARGING", 1, t += 1000);
    cl_flush(&e, t + 5, true, cb, NULL);
    assert(e.charge_open);
    printf("totals km %.1f kwh %.2f charged %.2f trips %u charges %u\n",
           e.total_km, e.total_kwh, e.total_charged_kwh, e.trips, e.charges);
    printf("state blob %zu bytes\nOK\n", sizeof(e));
    return 0;
}
