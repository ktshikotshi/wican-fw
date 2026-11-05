/*
 * cherylink.c: the WiCAN side of the CheryLink port. Glue only: the
 * logic is cl_engine.c (pure, host-tested).
 *
 *  autopid value sink -> queue -> "cherylink" task (PSRAM stack)
 *    -> cl_engine -> finished trips/charges as JSON lines in
 *       /data/cherylink/{drives,charges}.jsonl, the engine state in
 *       /data/cherylink/state.bin (every 10 min while changing, after
 *       every trip/charge, and before sleep)
 *    -> RANGE published back into autopid (ABRP est_battery_range,
 *       HA, MQTT) and CL_WH_KM (50 km Wh/km average).
 *
 * Every /data access goes through flash_jobs_run (the task and the HTTP
 * server run on PSRAM stacks, which cannot touch flash).
 *
 * Read-only by construction: this file never sends on the bus.
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include "cherylink.h"

#include <dirent.h>
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"

#include "autopid.h"
#include "flash_jobs.h"
#include "http_server_manager.h"
#include "rtc_manager.h"

#include "cl_engine.h"

static const char *TAG = "cherylink";

#define CL_DIR          "/data/cherylink"
#define CL_STATE        CL_DIR "/state.bin"
#define CL_STATE_TMP    CL_DIR "/state.tmp"
#define CL_DRIVES       CL_DIR "/drives.jsonl"
#define CL_DRIVES_OLD   CL_DIR "/drives.old.jsonl"
#define CL_CHARGES      CL_DIR "/charges.jsonl"
#define CL_CHARGES_OLD  CL_DIR "/charges.old.jsonl"
#define CL_ROUTES       CL_DIR "/routes"
#define CL_ROUTES_MAX   300 /* route files kept (oldest deleted) */
#define CL_ROTATE_BYTES (128 * 1024)
#define CL_QUEUE_LEN    64
#define CL_SAVE_MS      (10 * 60 * 1000)
#define CL_TICK_MS      5000
#define CL_PUBLISH_MS   15000
#define CL_PENDING_MAX  4
#define CL_LINE_MAX     768

typedef struct
{
    int16_t key;
    double  value;
    double  t;
} cl_item_t;

static QueueHandle_t     s_queue;
static SemaphoreHandle_t s_lock;
static TaskHandle_t      s_task;
static StaticTask_t      s_tcb;
static StackType_t       s_stack[6144] EXT_RAM_BSS_ATTR;

static cl_engine_t s_engine EXT_RAM_BSS_ATTR;
static cl_engine_t s_snap EXT_RAM_BSS_ATTR; /* HTTP copy */

static char s_pending[CL_PENDING_MAX][CL_LINE_MAX] EXT_RAM_BSS_ATTR;
static bool s_pending_charge[CL_PENDING_MAX];

/* a finished drive's route, written as /data/cherylink/routes/<start>.bin
 * (int32 lat, lon in 1e-6 degrees, little endian) */
typedef struct
{
    uint32_t start;
    uint32_t n;
    int32_t  pts[CL_ROUTE_MAX][2];
} cl_route_t;
static cl_route_t s_pending_route[CL_PENDING_MAX] EXT_RAM_BSS_ATTR;
static int  s_pending_n;

static volatile uint32_t s_dropped_full, s_dropped_clock, s_samples;
static uint32_t s_write_errors, s_saves;
static bool s_loaded, s_dirty;

/* ---- small helpers ------------------------------------------------------- */

static double now_s(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + tv.tv_usec / 1e6;
}

/* appends `"key":<num|null>` (with a leading comma unless first) */
static int jnum(char *b, size_t cap, size_t at, const char *key, double v,
                int prec, bool first)
{
    if (at + 1 >= cap)
    {
        return 0;
    }
    int w;
    if (isnan(v) || isinf(v))
    {
        w = snprintf(b + at, cap - at, "%s\"%s\":null", first ? "" : ",", key);
    }
    else
    {
        w = snprintf(b + at, cap - at, "%s\"%s\":%.*f", first ? "" : ",",
                     key, prec, v);
    }
    /* cut off: stop at cap - 1 so the sum never wraps */
    if (w < 0 || (size_t)w >= cap - at)
    {
        return (int)(cap - 1 - at);
    }
    return w;
}

#define JN(key, v, prec) n += jnum(buf, cap, n, key, v, prec, false)

/* printf-append into buf[cap] at *n. *n never passes cap - 1, so a later
 * append can never compute a wrapped (huge) size; a cut-off result leaves
 * *n == cap - 1, which callers treat as overflow. */
static void app(char *buf, size_t cap, size_t *n, const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));
static void app(char *buf, size_t cap, size_t *n, const char *fmt, ...)
{
    if (*n >= cap - 1)
    {
        *n = cap - 1;
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(buf + *n, cap - *n, fmt, ap);
    va_end(ap);
    if (w < 0 || (size_t)w >= cap - *n)
    {
        *n = cap - 1;
    }
    else
    {
        *n += (size_t)w;
    }
}

static size_t trip_json(char *buf, size_t cap, const cl_trip_t *t,
                        const cl_engine_t *e)
{
    size_t n = 0;
    app(buf, cap, &n, "{\"start\":%.0f,\"end\":%.0f", t->start,
                         t->end);
    JN("km", t->km, 2);
    JN("kwh", t->kwh, 3);
    JN("regen_kwh", t->regen_kwh, 3);
    JN("wh_km", t->km > 0.05 ? t->kwh * 1000 / t->km : NAN, 0);
    JN("soc_start", t->soc_start, 1);
    JN("soc_end", t->soc_end, 1);
    JN("odo_start", t->odo_start, 1);
    JN("odo_end", t->odo_end, 1);
    JN("max_speed", t->max_speed, 0);
    JN("avg_temp", t->avg_temp, 1);
    JN("hvac_kwh", t->hvac_kwh, 3);
    JN("start_lat", e->trip_start_lat, 6);
    JN("start_lon", e->trip_start_lon, 6);
    JN("end_lat", e->trip_end_lat, 6);
    JN("end_lon", e->trip_end_lon, 6);
    JN("route_pts", (double)e->route_n, 0);
    if (n < cap)
    {
        app(buf, cap, &n, "}");
    }
    return n < cap ? n : cap - 1;
}

static size_t charge_json(char *buf, size_t cap, const cl_charge_t *c,
                          const cl_engine_t *e)
{
    size_t n = 0;
    app(buf, cap, &n, "{\"start\":%.0f,\"end\":%.0f,\"dc\":%s",
                                c->start, c->end, c->dc ? "true" : "false");
    JN("kwh", c->kwh, 3);
    JN("kwh_vi", c->kwh_vi, 3);
    JN("kwh_counter", c->kwh_counter, 3);
    JN("soc_start", c->soc_start, 1);
    JN("soc_end", c->soc_end, 1);
    JN("peak_kw", c->peak_kw, 1);
    JN("avg_kw", c->end - c->start > 60
                     ? c->kwh / ((c->end - c->start) / 3600.0) : NAN, 1);
    JN("avg_temp", c->avg_temp, 1);
    JN("odo", c->odo, 1);
    JN("lat", e->charge_lat, 6);
    JN("lon", e->charge_lon, 6);
    if (n < cap)
    {
        app(buf, cap, &n, "}");
    }
    return n < cap ? n : cap - 1;
}

/* ---- flash jobs ------------------------------------------------------------ */

static esp_err_t job_load(void *arg)
{
    cl_engine_t *e = arg;
    if (mkdir(CL_DIR, 0775) != 0 && errno != EEXIST)
    {
        ESP_LOGW(TAG, "mkdir %s: %d", CL_DIR, errno);
    }
    if (mkdir(CL_ROUTES, 0775) != 0 && errno != EEXIST)
    {
        ESP_LOGW(TAG, "mkdir %s: %d", CL_ROUTES, errno);
    }
    FILE *f = fopen(CL_STATE, "rb");
    if (f == NULL)
    {
        return ESP_ERR_NOT_FOUND;
    }
    size_t got = fread(e, 1, sizeof(*e), f);
    int extra = fgetc(f);
    fclose(f);
    if (extra != EOF || e->magic != CL_MAGIC)
    {
        return ESP_ERR_INVALID_VERSION;
    }
    if (e->version == 1 && got == CL_V1_SIZE)
    {
        cl_engine_init_v2(e); /* history kept, location starts now */
    }
    else if (got != sizeof(*e) || e->version != CL_VERSION)
    {
        return ESP_ERR_INVALID_VERSION;
    }
    if (!cl_engine_sanitize(e))
    {
        ESP_LOGW(TAG, "state.bin had out-of-range indexes, clamped");
    }
    return ESP_OK;
}

static esp_err_t job_save(void *arg)
{
    const cl_engine_t *e = arg;
    FILE *f = fopen(CL_STATE_TMP, "wb");
    if (f == NULL)
    {
        return ESP_FAIL;
    }
    size_t put = fwrite(e, 1, sizeof(*e), f);
    int rc = fclose(f);
    if (put != sizeof(*e) || rc != 0)
    {
        unlink(CL_STATE_TMP);
        return ESP_FAIL;
    }
    /* LittleFS rename replaces the target atomically */
    return rename(CL_STATE_TMP, CL_STATE) == 0 ? ESP_OK : ESP_FAIL;
}

typedef struct
{
    const char *path, *old;
    const char *line;
} append_job_t;

static esp_err_t job_append(void *arg)
{
    const append_job_t *j = arg;
    struct stat st;
    if (stat(j->path, &st) == 0 && st.st_size >= CL_ROTATE_BYTES)
    {
        unlink(j->old);
        rename(j->path, j->old);
    }
    FILE *f = fopen(j->path, "a");
    if (f == NULL)
    {
        return ESP_FAIL;
    }
    int a = fputs(j->line, f);
    int b = fputc('\n', f);
    int c = fclose(f);
    return (a >= 0 && b != EOF && c == 0) ? ESP_OK : ESP_FAIL;
}

typedef struct
{
    const char *path, *old;
    char       *buf;
    size_t      cap, len;
} read_job_t;

static void read_into(read_job_t *j, const char *path)
{
    FILE *f = fopen(path, "r");
    if (f == NULL)
    {
        return;
    }
    while (j->len + 1 < j->cap)
    {
        size_t got = fread(j->buf + j->len, 1, j->cap - 1 - j->len, f);
        if (got == 0)
        {
            break;
        }
        j->len += got;
    }
    fclose(f);
}

static esp_err_t job_read(void *arg)
{
    read_job_t *j = arg;
    j->len = 0;
    read_into(j, j->old); /* older first: newest lines end the buffer */
    read_into(j, j->path);
    j->buf[j->len] = '\0';
    return ESP_OK;
}

static esp_err_t job_route_write(void *arg)
{
    const cl_route_t *r = arg;
    char path[64];
    snprintf(path, sizeof(path), CL_ROUTES "/%010lu.bin",
             (unsigned long)r->start);
    FILE *f = fopen(path, "wb");
    if (f == NULL)
    {
        return ESP_FAIL;
    }
    size_t put = fwrite(r->pts, sizeof(r->pts[0]), r->n, f);
    int rc = fclose(f);
    if (put != r->n || rc != 0)
    {
        unlink(path);
        return ESP_FAIL;
    }
    /* retention: names are zero-padded epochs, so the smallest is oldest */
    for (;;)
    {
        DIR *d = opendir(CL_ROUTES);
        if (d == NULL)
        {
            break;
        }
        unsigned count = 0;
        char oldest[32] = "";
        struct dirent *de;
        while ((de = readdir(d)) != NULL)
        {
            if (strstr(de->d_name, ".bin") == NULL)
            {
                continue;
            }
            count++;
            if (oldest[0] == '\0' || strcmp(de->d_name, oldest) < 0)
            {
                strlcpy(oldest, de->d_name, sizeof(oldest));
            }
        }
        closedir(d);
        if (count <= CL_ROUTES_MAX || oldest[0] == '\0')
        {
            break;
        }
        snprintf(path, sizeof(path), CL_ROUTES "/%s", oldest);
        unlink(path);
    }
    return ESP_OK;
}

typedef struct
{
    uint32_t start;
    int32_t (*pts)[2];
    size_t   max, n;
} route_read_t;

static esp_err_t job_route_read(void *arg)
{
    route_read_t *r = arg;
    char path[64];
    snprintf(path, sizeof(path), CL_ROUTES "/%010lu.bin",
             (unsigned long)r->start);
    FILE *f = fopen(path, "rb");
    if (f == NULL)
    {
        return ESP_ERR_NOT_FOUND;
    }
    r->n = fread(r->pts, sizeof(r->pts[0]), r->max, f);
    fclose(f);
    return ESP_OK;
}

/* ---- engine side ------------------------------------------------------------- */

/* called by the engine under s_lock: format only, write after unlock */
static void on_event(cl_evt_type_t type, const void *rec, void *ctx)
{
    (void)ctx;
    if (s_pending_n >= CL_PENDING_MAX)
    {
        ESP_LOGW(TAG, "pending full, record dropped");
        return;
    }
    char *line = s_pending[s_pending_n];
    if (type == CL_EVT_TRIP)
    {
        trip_json(line, CL_LINE_MAX, rec, &s_engine);
        s_pending_charge[s_pending_n] = false;
        /* the route is still in the engine until the next drive opens */
        cl_route_t *r = &s_pending_route[s_pending_n];
        r->start = (uint32_t)((const cl_trip_t *)rec)->start;
        r->n = s_engine.route_n <= CL_ROUTE_MAX ? s_engine.route_n : 0;
        for (uint32_t i = 0; i < r->n; i++)
        {
            r->pts[i][0] = (int32_t)lround(s_engine.route[i][0] * 1e6);
            r->pts[i][1] = (int32_t)lround(s_engine.route[i][1] * 1e6);
        }
        ESP_LOGI(TAG, "drive ended: %s", line);
    }
    else
    {
        charge_json(line, CL_LINE_MAX, rec, &s_engine);
        s_pending_charge[s_pending_n] = true;
        ESP_LOGI(TAG, "charge ended: %s", line);
    }
    s_pending_n++;
}

static void save_state(void)
{
    /* snapshot under the lock, write without it */
    static cl_engine_t copy EXT_RAM_BSS_ATTR;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    copy = s_engine;
    s_dirty = false;
    xSemaphoreGive(s_lock);
    if (flash_jobs_run(job_save, &copy) == ESP_OK)
    {
        s_saves++;
    }
    else
    {
        s_write_errors++;
        ESP_LOGW(TAG, "state save failed");
    }
}

/* writes the records the engine produced (caller does NOT hold s_lock) */
static void write_pending(void)
{
    static char lines[CL_PENDING_MAX][CL_LINE_MAX] EXT_RAM_BSS_ATTR;
    static cl_route_t routes[CL_PENDING_MAX] EXT_RAM_BSS_ATTR;
    bool charge[CL_PENDING_MAX];
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int n = s_pending_n;
    for (int i = 0; i < n; i++)
    {
        memcpy(lines[i], s_pending[i], CL_LINE_MAX);
        charge[i] = s_pending_charge[i];
        if (!charge[i])
        {
            routes[i].start = s_pending_route[i].start;
            routes[i].n = s_pending_route[i].n;
            memcpy(routes[i].pts, s_pending_route[i].pts,
                   routes[i].n * sizeof(routes[i].pts[0]));
        }
    }
    s_pending_n = 0;
    xSemaphoreGive(s_lock);
    for (int i = 0; i < n; i++)
    {
        append_job_t j =
        {
            .path = charge[i] ? CL_CHARGES : CL_DRIVES,
            .old = charge[i] ? CL_CHARGES_OLD : CL_DRIVES_OLD,
            .line = lines[i],
        };
        if (flash_jobs_run(job_append, &j) != ESP_OK)
        {
            s_write_errors++;
            ESP_LOGW(TAG, "append %s failed", j.path);
        }
        if (!charge[i] && routes[i].n > 0 &&
            flash_jobs_run(job_route_write, &routes[i]) != ESP_OK)
        {
            s_write_errors++;
            ESP_LOGW(TAG, "route write failed");
        }
    }
    if (n > 0)
    {
        save_state();
    }
}

static void publish(void)
{
    double t = now_s();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool soc_fresh = !isnan(cl_fresh(&s_engine, CL_SOC, t, 300));
    double range = cl_est_range_km(&s_engine);
    double whkm = cl_range_wh_km(&s_engine);
    xSemaphoreGive(s_lock);
    if (soc_fresh && !isnan(range))
    {
        (void)autopid_publish_external("RANGE", "km", range);
        (void)autopid_publish_external("CL_WH_KM", "Wh/km", round(whkm));
    }
}

static void task_main(void *arg)
{
    (void)arg;
    TickType_t last_tick = xTaskGetTickCount();
    TickType_t last_save = last_tick, last_pub = last_tick;
    for (;;)
    {
        cl_item_t it;
        if (xQueueReceive(s_queue, &it, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            do
            {
                cl_ingest(&s_engine, it.key, it.value, it.t, on_event, NULL);
            } while (xQueueReceive(s_queue, &it, 0) == pdTRUE);
            s_dirty = true;
            xSemaphoreGive(s_lock);
        }
        TickType_t now = xTaskGetTickCount();
        if (now - last_tick >= pdMS_TO_TICKS(CL_TICK_MS) &&
            rtc_manager_time_valid())
        {
            last_tick = now;
            xSemaphoreTake(s_lock, portMAX_DELAY);
            cl_tick(&s_engine, now_s(), on_event, NULL);
            xSemaphoreGive(s_lock);
        }
        write_pending();
        if (now - last_pub >= pdMS_TO_TICKS(CL_PUBLISH_MS))
        {
            last_pub = now;
            publish();
        }
        if (s_dirty && now - last_save >= pdMS_TO_TICKS(CL_SAVE_MS))
        {
            last_save = now;
            save_state();
        }
    }
}

/* ---- public lifecycle -------------------------------------------------------- */

void cherylink_autopid_sink(const char *name, const char *unit, double value,
                            bool changed)
{
    (void)unit;
    (void)changed;
    if (s_queue == NULL)
    {
        return;
    }
    int key = cl_key_from_name(name);
    if (key < 0)
    {
        return;
    }
    if (!rtc_manager_time_valid())
    {
        s_dropped_clock++;
        return;
    }
    cl_item_t it = { .key = (int16_t)key, .value = value, .t = now_s() };
    if (xQueueSend(s_queue, &it, 0) != pdTRUE)
    {
        s_dropped_full++;
        return;
    }
    s_samples++;
}

esp_err_t cherylink_init(void)
{
    if (s_queue != NULL)
    {
        return ESP_OK;
    }
    s_queue = xQueueCreateWithCaps(CL_QUEUE_LEN, sizeof(cl_item_t),
                                   MALLOC_CAP_SPIRAM);
    s_lock = xSemaphoreCreateMutex();
    if (s_queue == NULL || s_lock == NULL)
    {
        return ESP_ERR_NO_MEM;
    }
    cl_engine_init(&s_engine);
    return ESP_OK;
}

esp_err_t cherylink_start(void)
{
    if (s_queue == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_task != NULL)
    {
        return ESP_OK;
    }
    static cl_engine_t loaded EXT_RAM_BSS_ATTR;
    esp_err_t err = flash_jobs_run(job_load, &loaded);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (err == ESP_OK)
    {
        s_engine = loaded;
        s_loaded = true;
        ESP_LOGI(TAG, "state loaded: %lu drives %.1f km, %lu charges",
                 (unsigned long)s_engine.trips, s_engine.total_km,
                 (unsigned long)s_engine.charges);
    }
    else
    {
        cl_engine_init(&s_engine);
        ESP_LOGI(TAG, "fresh state (%s)", esp_err_to_name(err));
    }
    xSemaphoreGive(s_lock);
    s_task = xTaskCreateStatic(task_main, "cherylink",
                               sizeof(s_stack) / sizeof(s_stack[0]), NULL, 2,
                               s_stack, &s_tcb);
    return s_task != NULL ? ESP_OK : ESP_FAIL;
}

esp_err_t cherylink_stop(void)
{
    if (s_queue == NULL || s_task == NULL)
    {
        return ESP_OK;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    cl_item_t it;
    while (xQueueReceive(s_queue, &it, 0) == pdTRUE)
    {
        cl_ingest(&s_engine, it.key, it.value, it.t, on_event, NULL);
    }
    if (rtc_manager_time_valid())
    {
        /* a trip ends at its last data; a charge carries on after wake */
        cl_flush(&s_engine, now_s(), true, on_event, NULL);
    }
    xSemaphoreGive(s_lock);
    write_pending();
    save_state();
    ESP_LOGI(TAG, "state saved for sleep");
    return ESP_OK;
}

/* ---- HTTP ------------------------------------------------------------------------ */

extern const char cl_page_start[] asm("_binary_cherylink_html_start");
extern const char cl_page_end[] asm("_binary_cherylink_html_end");

static esp_err_t page_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, cl_page_start, cl_page_end - cl_page_start);
}

static void band_json(char *buf, size_t cap, size_t *n, const char *key,
                      const cl_band_t *b, int count)
{
    app(buf, cap, n, ",\"%s\":[", key);
    for (int i = 0; i < count && *n < cap; i++)
    {
        app(buf, cap, n, "%s{\"km\":%.2f,\"wh_km\":",
                       i ? "," : "", b[i].km);
        if (b[i].km > 0.4 && *n < cap)
        {
            app(buf, cap, n, "%.0f}", b[i].wh / b[i].km);
        }
        else if (*n < cap)
        {
            app(buf, cap, n, "null}");
        }
    }
    if (*n < cap)
    {
        app(buf, cap, n, "]");
    }
}

static esp_err_t status_get(httpd_req_t *req)
{
    const size_t cap = 8192;
    char *buf = heap_caps_malloc(cap, MALLOC_CAP_SPIRAM);
    if (buf == NULL)
    {
        return httpd_resp_send_500(req);
    }
    bool clock = rtc_manager_time_valid();
    double t = now_s();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_snap = s_engine;
    xSemaphoreGive(s_lock);
    const cl_engine_t *e = &s_snap;

    size_t n = 0;
    app(buf, cap, &n, "{\"now\":%.0f,\"clock_valid\":%s,\"loaded\":%s,\"samples\":%lu,"
        "\"dropped\":%lu,\"dropped_clock\":%lu,\"write_errors\":%lu,"
        "\"saves\":%lu,\"pack_kwh\":%.3f",
        t, clock ? "true" : "false", s_loaded ? "true" : "false",
        (unsigned long)s_samples, (unsigned long)s_dropped_full,
        (unsigned long)s_dropped_clock, (unsigned long)s_write_errors,
        (unsigned long)s_saves, CL_PACK_KWH);
    JN("stack_free", s_task ? (double)uxTaskGetStackHighWaterMark(s_task) : NAN,
       0);

    /* live values: {"SOC":[value, age_s], ...} only the ones seen */
    app(buf, cap, &n, ",\"values\":{");
    bool first = true;
    for (int k = 0; k < CL_KEY_COUNT && n < cap; k++)
    {
        if (e->at[k] <= 0)
        {
            continue;
        }
        app(buf, cap, &n, "%s\"%s\":[%.3f,%.0f]",
                      first ? "" : ",", cl_key_name(k), e->val[k],
                      t - e->at[k]);
        first = false;
    }
    if (n < cap)
    {
        app(buf, cap, &n, "}");
    }

    {
        bool fresh = !isnan(e->lat) && t - e->lat_at < 30;
        app(buf, cap, &n, ",\"gps\":{\"fresh\":%s",
                      fresh ? "true" : "false");
        JN("lat", e->lat, 6);
        JN("lon", e->lon, 6);
        JN("age", e->lat_at > 0 ? t - e->lat_at : NAN, 0);
        if (n < cap)
        {
            app(buf, cap, &n, "}");
        }
    }
    JN("est_range_km", cl_est_range_km(e), 0);
    JN("range_wh_km", cl_range_wh_km(e), 0);
    JN("range_km_seen", e->range_count + e->range_part_km, 1);
    JN("avg_wh_km_5", cl_avg_wh_km(e, 5), 0);
    JN("instant_wh_km", cl_instant_wh_km(e), 0);
    JN("kw", e->kw_last_t > 0 ? e->kw_last : NAN, 2);

    static float bars[CL_BARS];
    size_t nb = cl_bars(e, bars, CL_BARS);
    app(buf, cap, &n, ",\"bars\":[");
    for (size_t i = 0; i < nb && n < cap; i++)
    {
        app(buf, cap, &n, "%s%.0f", i ? "," : "", bars[i]);
    }
    if (n < cap)
    {
        app(buf, cap, &n, "]");
    }

    band_json(buf, cap, &n, "by_speed", e->by_speed, CL_SPEED_BANDS);
    band_json(buf, cap, &n, "by_temp", e->by_temp, CL_TEMP_BANDS);

    char rec[CL_LINE_MAX];
    if (e->trip_open)
    {
        trip_json(rec, sizeof(rec), &e->trip, e);
        app(buf, cap, &n, ",\"trip\":%s", rec);
    }
    else
    {
        app(buf, cap, &n, ",\"trip\":null");
    }
    if (e->charge_open)
    {
        cl_charge_t c = e->charge;
        c.kwh = c.kwh_counter >= CL_MIN_CHARGE_KWH ? c.kwh_counter : c.kwh_vi;
        charge_json(rec, sizeof(rec), &c, e);
        app(buf, cap, &n, ",\"charge\":%s", rec);
    }
    else
    {
        app(buf, cap, &n, ",\"charge\":null");
    }

    app(buf, cap, &n,
                  ",\"totals\":{\"since\":%.0f,\"km\":%.1f,\"kwh\":%.2f,"
                  "\"regen_kwh\":%.2f,\"charged_kwh\":%.2f,\"drives\":%lu,"
                  "\"charges\":%lu}}",
                  e->since, e->total_km, e->total_kwh, e->total_regen_kwh,
                  e->total_charged_kwh, (unsigned long)e->trips,
                  (unsigned long)e->charges);

    if (n >= cap - 1)
    {
        free(buf);
        ESP_LOGW(TAG, "status JSON over %u B", (unsigned)cap);
        return httpd_resp_send_500(req);
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_send(req, buf, (ssize_t)n);
    free(buf);
    return err;
}

/* newest-first JSON array of the last ?limit= lines of a history file */
static esp_err_t history_get(httpd_req_t *req, const char *path,
                             const char *old)
{
    int limit = 50;
    char q[48], v[12];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
        httpd_query_key_value(q, "limit", v, sizeof(v)) == ESP_OK)
    {
        limit = atoi(v);
    }
    if (limit < 1)
    {
        limit = 1;
    }
    if (limit > 2000)
    {
        limit = 2000;
    }
    read_job_t j =
    {
        .path = path, .old = old,
        .cap = 2 * CL_ROTATE_BYTES + CL_LINE_MAX * 4,
    };
    j.buf = heap_caps_malloc(j.cap, MALLOC_CAP_SPIRAM);
    if (j.buf == NULL)
    {
        return httpd_resp_send_500(req);
    }
    if (flash_jobs_run(job_read, &j) != ESP_OK)
    {
        free(j.buf);
        return httpd_resp_send_500(req);
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send_chunk(req, "[", 1);
    int sent = 0;
    size_t end = j.len;
    while (end > 0 && sent < limit)
    {
        /* skip trailing newlines, find the line start */
        while (end > 0 && (j.buf[end - 1] == '\n' || j.buf[end - 1] == '\r'))
        {
            end--;
        }
        size_t start = end;
        while (start > 0 && j.buf[start - 1] != '\n')
        {
            start--;
        }
        if (end > start && j.buf[start] == '{' && j.buf[end - 1] == '}')
        {
            if (sent > 0)
            {
                httpd_resp_send_chunk(req, ",", 1);
            }
            httpd_resp_send_chunk(req, j.buf + start, (ssize_t)(end - start));
            sent++;
        }
        end = start;
    }
    free(j.buf);
    httpd_resp_send_chunk(req, "]", 1);
    return httpd_resp_send_chunk(req, NULL, 0);
}

/* GET /api/cherylink/route?start=<drive start epoch> -> [[lat,lon],...];
 * the open drive's route when start matches it */
static esp_err_t route_get(httpd_req_t *req)
{
    char q[48], v[16];
    uint32_t start = 0;
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
        httpd_query_key_value(q, "start", v, sizeof(v)) == ESP_OK)
    {
        start = (uint32_t)strtoul(v, NULL, 10);
    }
    route_read_t r = { .start = start, .max = CL_ROUTE_MAX };
    r.pts = heap_caps_malloc(CL_ROUTE_MAX * sizeof(r.pts[0]),
                             MALLOC_CAP_SPIRAM);
    if (r.pts == NULL)
    {
        return httpd_resp_send_500(req);
    }
    esp_err_t err = ESP_ERR_NOT_FOUND;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_engine.trip_open && (uint32_t)s_engine.trip.start == start)
    {
        r.n = s_engine.route_n <= CL_ROUTE_MAX ? s_engine.route_n : 0;
        for (size_t i = 0; i < r.n; i++)
        {
            r.pts[i][0] = (int32_t)lround(s_engine.route[i][0] * 1e6);
            r.pts[i][1] = (int32_t)lround(s_engine.route[i][1] * 1e6);
        }
        err = ESP_OK;
    }
    xSemaphoreGive(s_lock);
    if (err != ESP_OK && start > 0)
    {
        err = flash_jobs_run(job_route_read, &r);
    }
    if (err != ESP_OK)
    {
        free(r.pts);
        httpd_resp_set_status(req, "404 Not Found");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "[]");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    char chunk[512];
    size_t c = 0;
    chunk[c++] = '[';
    for (size_t i = 0; i < r.n; i++)
    {
        if (c > sizeof(chunk) - 40)
        {
            httpd_resp_send_chunk(req, chunk, (ssize_t)c);
            c = 0;
        }
        c += (size_t)snprintf(chunk + c, sizeof(chunk) - c, "%s[%.6f,%.6f]",
                              i ? "," : "", r.pts[i][0] / 1e6,
                              r.pts[i][1] / 1e6);
    }
    chunk[c++] = ']';
    httpd_resp_send_chunk(req, chunk, (ssize_t)c);
    free(r.pts);
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t drives_get(httpd_req_t *req)
{
    return history_get(req, CL_DRIVES, CL_DRIVES_OLD);
}

static esp_err_t charges_get(httpd_req_t *req)
{
    return history_get(req, CL_CHARGES, CL_CHARGES_OLD);
}

esp_err_t cherylink_register_http(void)
{
    static const httpd_uri_t URIS[] =
    {
        { .uri = "/cherylink", .method = HTTP_GET, .handler = page_get },
        { .uri = "/api/cherylink", .method = HTTP_GET, .handler = status_get },
        { .uri = "/api/cherylink/drives", .method = HTTP_GET,
          .handler = drives_get },
        { .uri = "/api/cherylink/charges", .method = HTTP_GET,
          .handler = charges_get },
        { .uri = "/api/cherylink/route", .method = HTTP_GET,
          .handler = route_get },
    };
    return http_server_manager_register_handlers(
        URIS, sizeof(URIS) / sizeof(URIS[0]));
}
