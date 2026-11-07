/*
 * cl_ble: the CheryLink phone feed and the ABRP handover.
 *
 *  - Bluetooth: channel FFF7 out (indicate) / FFF8 in on the WiCAN FFF0
 *    service (task cl_ble).
 *  - Wi-Fi: a TCP stream on port CL_TCP_PORT, same bytes both ways
 *    (task cl_tcp), for the app's "WiCAN Wi-Fi" source.
 *
 * Protocol logic lives in cl_ble_core.c (host-tested); wire format in
 * BLE_PROTOCOL.md. WiCAN polls the car (autopid); this only forwards what
 * autopid already read, so the phone puts nothing on the bus. While a
 * phone says it sends to ABRP (hello / heartbeat with "abrp":true),
 * WiCAN's own ABRP destination is held; CLB_HOLD_S after the phone's last
 * message (it left the car) WiCAN sends to ABRP again over its uplink.
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include "cherylink.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "lwip/sockets.h"

#include "esp_app_desc.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "ble_manager.h"
#include "data_destinations.h"
#include "rtc_manager.h"

#include "cl_ble_core.h"
#include "cl_engine.h"
#include "cl_internal.h"

static const char *TAG = "cherylink_ble";

#define CLB_UUID_OUT     0xFFF7
#define CLB_UUID_IN      0xFFF8
#define CL_TCP_PORT      35001
#define CLB_VALUES_S     1.0
#define CLB_STATUS_S     10.0
#define CLB_FRAME_MAX    1024
#define CLB_RECORDS      4
#define CLB_REC_MAX      1024

/* ---- shared ----------------------------------------------------------------- */

static clb_t             s_tab EXT_RAM_BSS_ATTR;
static SemaphoreHandle_t s_mx;
static volatile bool     s_run;

/* finished drives / charges (the newest CLB_RECORDS, ids from 1) */
static char     s_rec[CLB_RECORDS][CLB_REC_MAX] EXT_RAM_BSS_ATTR;
static bool     s_rec_charge[CLB_RECORDS];
static uint32_t s_rec_id[CLB_RECORDS];
static uint32_t s_rec_last; /* id of the newest record, 0 = none */

static volatile double s_pair_until; /* mono s, 0 = no timed window */
static volatile bool   s_hold_now;

/* one phone connection */
typedef struct
{
    const char *name;
    clb_sub_t   sub;
    bool        up;
    double      next_values, next_status;
    uint32_t    rec_sent;   /* newest record id this link has had */
    uint32_t    frames, tx_errors;
    /* returns false on a write error (link gone) */
    bool (*write)(void *ctx, const char *buf, size_t n);
    void       *ctx;
} cl_sess_t;

static cl_sess_t s_ble_sess EXT_RAM_BSS_ATTR;
static cl_sess_t s_tcp_sess EXT_RAM_BSS_ATTR;

/* ---- Bluetooth task --------------------------------------------------------- */

static uint8_t      s_rx_storage[2048] EXT_RAM_BSS_ATTR;
static int          s_ch = -1;
static TaskHandle_t s_task;
static StaticTask_t s_tcb;
static StackType_t  s_stack[6144] EXT_RAM_BSS_ATTR;

/* ---- Wi-Fi task ------------------------------------------------------------- */

static TaskHandle_t s_tcp_task;
static StaticTask_t s_tcp_tcb;
static StackType_t  s_tcp_stack[6144] EXT_RAM_BSS_ATTR;
static volatile int s_tcp_client = -1;
static volatile bool s_tcp_bad;   /* a write failed: drop the client */

static double mono_s(void)
{
    return esp_timer_get_time() / 1e6;
}

static double wall_s(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec;
}

void cl_ble_on_value(const char *name, double value)
{
    if (s_mx == NULL)
    {
        return;
    }
    /* poller context: never wait long; a missed update is re-sent with
       the next sample of that name */
    if (xSemaphoreTake(s_mx, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        clb_put(&s_tab, name, value);
        xSemaphoreGive(s_mx);
    }
}

void cl_ble_on_record(bool charge, const char *line)
{
    if (s_mx == NULL || line == NULL)
    {
        return;
    }
    xSemaphoreTake(s_mx, portMAX_DELAY);
    int slot = (int)(s_rec_last % CLB_RECORDS);
    s_rec_last++;
    strlcpy(s_rec[slot], line, sizeof(s_rec[0]));
    s_rec_charge[slot] = charge;
    s_rec_id[slot] = s_rec_last;
    xSemaphoreGive(s_mx);
}

/* WiCAN's ABRP sender holds while ANY phone owns ABRP */
static void update_hold(double now)
{
    xSemaphoreTake(s_mx, portMAX_DELAY);
    bool h = clb_abrp_hold(&s_ble_sess.sub, now) ||
             clb_abrp_hold(&s_tcp_sess.sub, now);
    bool changed = h != s_hold_now;
    s_hold_now = h;
    xSemaphoreGive(s_mx);
    if (changed)
    {
        data_destinations_set_abrp_hold(h);
        ESP_LOGI(TAG, "ABRP sender: %s", h ? "phone" : "WiCAN");
    }
}

static bool sess_send(cl_sess_t *s, const char *buf, size_t n)
{
    if (n == 0)
    {
        return true;
    }
    if (!s->write(s->ctx, buf, n))
    {
        s->tx_errors++;
        return false;
    }
    s->frames++;
    return true;
}

static void sess_rx(cl_sess_t *s, const uint8_t *d, size_t n, double now)
{
    xSemaphoreTake(s_mx, portMAX_DELAY);
    clb_rx(&s->sub, d, n, now);
    xSemaphoreGive(s_mx);
}

static void sess_down(cl_sess_t *s)
{
    if (s->up)
    {
        ESP_LOGI(TAG, "%s: phone link down", s->name);
    }
    s->up = false;
    xSemaphoreTake(s_mx, portMAX_DELAY);
    clb_link_down(&s->sub);
    xSemaphoreGive(s_mx);
}

static void send_hello(cl_sess_t *s, char *buf, size_t cap)
{
    const esp_app_desc_t *app = esp_app_get_description();
    int w = snprintf(buf, cap,
                     "{\"t\":\"hello\",\"proto\":%d,\"fw\":\"%s\","
                     "\"link\":\"%s\",\"pack_kwh\":%.3f,\"clock_valid\":%s,"
                     "\"abrp\":\"%s\"}\n",
                     CLB_PROTO, app->version, s->name, CL_PACK_KWH,
                     rtc_manager_time_valid() ? "true" : "false",
                     s_hold_now ? "phone" : "wican");
    if (w > 0 && (size_t)w < cap)
    {
        sess_send(s, buf, (size_t)w);
    }
}

static void send_status(cl_sess_t *s, char *buf, size_t cap)
{
    int w = snprintf(buf, cap, "{\"t\":\"s\",\"ts\":%.0f,\"abrp\":\"%s\","
                               "\"s\":", wall_s(),
                     s_hold_now ? "phone" : "wican");
    if (w <= 0 || (size_t)w >= cap - 4)
    {
        return;
    }
    size_t n = (size_t)w;
    size_t body = cl_status_compact(buf + n, cap - n - 3);
    if (body == 0)
    {
        return;
    }
    n += body;
    memcpy(buf + n, "}\n", 3);
    sess_send(s, buf, n + 2);
}

static void send_records(cl_sess_t *s, char *buf, size_t cap)
{
    for (;;)
    {
        bool have = false;
        int w = 0;
        xSemaphoreTake(s_mx, portMAX_DELAY);
        if (s->rec_sent >= s_rec_last)
        {
            xSemaphoreGive(s_mx);
            return; /* nothing new */
        }
        /* oldest record this link has not had that is still kept */
        uint32_t want = s->rec_sent + 1;
        if (s_rec_last > CLB_RECORDS && want < s_rec_last - CLB_RECORDS + 1)
        {
            want = s_rec_last - CLB_RECORDS + 1;
        }
        int slot = (int)((want - 1) % CLB_RECORDS);
        w = snprintf(buf, cap, "{\"t\":\"%s\",\"r\":%s}\n",
                     s_rec_charge[slot] ? "charge" : "trip", s_rec[slot]);
        have = s_rec_id[slot] == want && w > 0 && (size_t)w < cap;
        s->rec_sent = want;
        xSemaphoreGive(s_mx);
        if (have && !sess_send(s, buf, (size_t)w))
        {
            return; /* lost to this link only: it is in the history API */
        }
    }
}

/* one pass for a connected, subscribed link */
static void sess_step(cl_sess_t *s, char *frame, size_t cap, double now)
{
    if (!s->up)
    {
        s->up = true;
        ESP_LOGI(TAG, "%s: phone connected", s->name);
        xSemaphoreTake(s_mx, portMAX_DELAY);
        clb_sub_resync(&s->sub);
        xSemaphoreGive(s_mx);
        send_hello(s, frame, cap);
        s->next_values = 0;
        s->next_status = 0;
    }
    if (now >= s->next_values)
    {
        s->next_values = now + CLB_VALUES_S;
        for (int i = 0; i < 8; i++) /* a full table spans a few frames */
        {
            xSemaphoreTake(s_mx, portMAX_DELAY);
            size_t n = clb_values_frame(&s_tab, &s->sub, frame, cap, wall_s());
            xSemaphoreGive(s_mx);
            if (n == 0 || !sess_send(s, frame, n))
            {
                break;
            }
        }
    }
    if (now >= s->next_status)
    {
        s->next_status = now + CLB_STATUS_S;
        send_status(s, frame, cap);
    }
    send_records(s, frame, cap);
}

/* ---- Bluetooth ---------------------------------------------------------------- */

static bool ble_write(void *ctx, const char *buf, size_t n)
{
    (void)ctx;
    return ble_manager_channel_write(s_ch, (const uint8_t *)buf, n) == (int)n;
}

static void task_main(void *arg)
{
    (void)arg;
    static char frame[CLB_FRAME_MAX] EXT_RAM_BSS_ATTR;
    uint8_t rx[128];
    cl_sess_t *s = &s_ble_sess;

    while (s_run)
    {
        int r = ble_manager_channel_read(s_ch, rx, sizeof(rx), 250);
        double now = mono_s();
        if (r > 0)
        {
            sess_rx(s, rx, (size_t)r, now);
        }
        else if (r < 0)
        {
            sess_down(s);
            vTaskDelay(pdMS_TO_TICKS(250)); /* no link: read returns at once */
        }

        update_hold(now);
        if (s_pair_until > 0 && now >= s_pair_until)
        {
            s_pair_until = 0;
            ble_manager_pairing_disable();
            ESP_LOGI(TAG, "pairing window closed");
        }

        bool up = ble_manager_is_secured() &&
                  ble_manager_channel_out_mode(s_ch) != BLE_MANAGER_CH_OUT_NONE;
        if (!up)
        {
            s->up = false;
            continue;
        }
        sess_step(s, frame, sizeof(frame), now);
    }
    s_task = NULL;
    vTaskDelete(NULL);
}

/* ---- Wi-Fi (TCP) ---------------------------------------------------------- */

static bool tcp_write(void *ctx, const char *buf, size_t n)
{
    (void)ctx;
    int fd = s_tcp_client;
    size_t off = 0;
    while (fd >= 0 && off < n)
    {
        int w = send(fd, buf + off, n - off, 0);
        if (w <= 0)
        {
            s_tcp_bad = true;
            return false;
        }
        off += (size_t)w;
    }
    return fd >= 0;
}

static int tcp_listen(void)
{
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0)
    {
        return -1;
    }
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in a =
    {
        .sin_family = AF_INET,
        .sin_port = htons(CL_TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(fd, (struct sockaddr *)&a, sizeof(a)) != 0 || listen(fd, 1) != 0)
    {
        close(fd);
        return -1;
    }
    return fd;
}

static void tcp_close_client(cl_sess_t *s)
{
    int fd = s_tcp_client;
    s_tcp_client = -1;
    if (fd >= 0)
    {
        shutdown(fd, SHUT_RDWR);
        close(fd);
        sess_down(s);
    }
}

static void tcp_task(void *arg)
{
    (void)arg;
    static char frame[CLB_FRAME_MAX] EXT_RAM_BSS_ATTR;
    uint8_t rx[128];
    cl_sess_t *s = &s_tcp_sess;
    int lfd = -1;

    while (s_run)
    {
        if (lfd < 0)
        {
            lfd = tcp_listen();
            if (lfd < 0)
            {
                vTaskDelay(pdMS_TO_TICKS(2000)); /* network not up yet */
                continue;
            }
            ESP_LOGI(TAG, "wifi: listening on tcp %d", CL_TCP_PORT);
        }

        fd_set rs;
        FD_ZERO(&rs);
        FD_SET(lfd, &rs);
        int maxfd = lfd;
        int cfd = s_tcp_client;
        if (cfd >= 0)
        {
            FD_SET(cfd, &rs);
            maxfd = cfd > maxfd ? cfd : maxfd;
        }
        struct timeval tv = { .tv_sec = 0, .tv_usec = 250000 };
        int sel = select(maxfd + 1, &rs, NULL, NULL, &tv);
        double now = mono_s();
        if (sel < 0)
        {
            close(lfd);
            lfd = -1;
            tcp_close_client(s);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (sel > 0 && FD_ISSET(lfd, &rs))
        {
            int nfd = accept(lfd, NULL, NULL);
            if (nfd >= 0)
            {
                /* one phone at a time: the newest connection wins */
                tcp_close_client(s);
                struct timeval sto = { .tv_sec = 5, .tv_usec = 0 };
                setsockopt(nfd, SOL_SOCKET, SO_SNDTIMEO, &sto, sizeof(sto));
                int one = 1;
                setsockopt(nfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
                s_tcp_bad = false;
                s_tcp_client = nfd;
            }
        }
        cfd = s_tcp_client;
        if (sel > 0 && cfd >= 0 && FD_ISSET(cfd, &rs))
        {
            int r = recv(cfd, rx, sizeof(rx), 0);
            if (r > 0)
            {
                sess_rx(s, rx, (size_t)r, now);
            }
            else
            {
                tcp_close_client(s); /* closed or failed */
            }
        }

        update_hold(now);
        if (s_tcp_client >= 0)
        {
            sess_step(s, frame, sizeof(frame), now);
            if (s_tcp_bad)
            {
                tcp_close_client(s);
            }
        }
    }
    tcp_close_client(s);
    if (lfd >= 0)
    {
        close(lfd);
    }
    s_tcp_task = NULL;
    vTaskDelete(NULL);
}

/* ---- surface ------------------------------------------------------------ */

bool cl_ble_pair_open(int seconds)
{
    if (!ble_manager_is_enabled() || s_task == NULL)
    {
        return false;
    }
    if (ble_manager_pairing_is_enabled() && s_pair_until <= 0)
    {
        return true; /* open by setting (pairing_at_boot): leave it so */
    }
    ble_manager_pairing_enable();
    s_pair_until = mono_s() + (seconds > 0 ? seconds : 120);
    ESP_LOGI(TAG, "pairing window open for %d s", seconds);
    return true;
}

void cl_ble_info(cl_ble_info_t *out)
{
    memset(out, 0, sizeof(*out));
    out->enabled = ble_manager_is_enabled();
    out->connected = ble_manager_is_secured();
    out->pairing = ble_manager_pairing_is_enabled();
    double left = s_pair_until > 0 ? s_pair_until - mono_s() : 0;
    out->pairing_left = left > 0 ? (int)left : 0;
    out->phone_abrp = s_hold_now;
    out->wifi_connected = s_tcp_client >= 0;
    if (s_mx != NULL && xSemaphoreTake(s_mx, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        strlcpy(out->app, s_ble_sess.sub.app[0] ? s_ble_sess.sub.app
                                                : s_tcp_sess.sub.app,
                sizeof(out->app));
        xSemaphoreGive(s_mx);
    }
}

esp_err_t cherylink_ble_start(void)
{
    if (s_mx == NULL)
    {
        s_mx = xSemaphoreCreateMutex();
        if (s_mx == NULL)
        {
            return ESP_ERR_NO_MEM;
        }
        clb_init(&s_tab);
        memset(&s_ble_sess, 0, sizeof(s_ble_sess));
        memset(&s_tcp_sess, 0, sizeof(s_tcp_sess));
        clb_sub_init(&s_ble_sess.sub);
        clb_sub_init(&s_tcp_sess.sub);
        s_ble_sess.name = "ble";
        s_ble_sess.write = ble_write;
        s_tcp_sess.name = "wifi";
        s_tcp_sess.write = tcp_write;
    }
    s_run = true;

    if (s_tcp_task == NULL)
    {
        s_tcp_task = xTaskCreateStatic(tcp_task, "cl_tcp",
                                       sizeof(s_tcp_stack) /
                                           sizeof(s_tcp_stack[0]),
                                       NULL, 2, s_tcp_stack, &s_tcp_tcb);
        if (s_tcp_task == NULL)
        {
            ESP_LOGW(TAG, "wifi feed task not started");
        }
    }

    if (s_task != NULL)
    {
        return ESP_OK;
    }
    if (s_ch < 0)
    {
        ble_manager_channel_desc_t desc =
        {
            .name = "cherylink",
            .uuid_out = CLB_UUID_OUT,
            .uuid_in = CLB_UUID_IN,
            .rx_storage = s_rx_storage,
            .rx_size = sizeof(s_rx_storage),
            .on_event = NULL,
            /* confirmed PDUs: the frames are small (about 1 KB/s) and
               the protocol then needs no credits or frame counter */
            .out_modes = BLE_MANAGER_CH_OUT_INDICATE,
        };
        esp_err_t err = ble_manager_channel_register(&desc, &s_ch);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "channel not registered: %s", esp_err_to_name(err));
            s_ch = -1;
            return ESP_OK; /* log-and-degrade: Wi-Fi feed + page still work */
        }
    }
    s_task = xTaskCreateStatic(task_main, "cl_ble",
                               sizeof(s_stack) / sizeof(s_stack[0]), NULL, 2,
                               s_stack, &s_tcb);
    if (s_task == NULL)
    {
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "started (channel %d, FFF7/FFF8; wifi tcp %d)", s_ch,
             CL_TCP_PORT);
    return ESP_OK;
}

esp_err_t cherylink_ble_stop(void)
{
    s_run = false;
    for (int i = 0; i < 50 && (s_task != NULL || s_tcp_task != NULL); i++)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    data_destinations_set_abrp_hold(false);
    return ESP_OK;
}
