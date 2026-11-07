/*
 * cl_ble: the CheryLink Bluetooth channel (FFF7 out / FFF8 in on the
 * WiCAN FFF0 service) and the ABRP handover. Protocol logic lives in
 * cl_ble_core.c (host-tested); wire format in BLE_PROTOCOL.md.
 *
 * WiCAN polls the car (autopid); this only forwards what autopid already
 * read, so the phone puts nothing on the bus. While the phone says it
 * sends to ABRP (hello / heartbeat with "abrp":true), WiCAN's own ABRP
 * destination is held; CLB_HOLD_S after the phone's last message (it
 * left the car) WiCAN sends to ABRP again over its own uplink (LTE).
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include "cherylink.h"

#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

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
#define CLB_VALUES_MS    1000
#define CLB_STATUS_MS    10000
#define CLB_FRAME_MAX    1024
#define CLB_RECORDS      4

static clb_t             s_core EXT_RAM_BSS_ATTR;
static SemaphoreHandle_t s_mx;
static uint8_t           s_rx_storage[2048] EXT_RAM_BSS_ATTR;
static int               s_ch = -1;
static volatile bool     s_run;
static TaskHandle_t      s_task;
static StaticTask_t      s_tcb;
static StackType_t       s_stack[6144] EXT_RAM_BSS_ATTR;

/* finished drives / charges waiting for the phone (newest kept) */
static char  s_rec[CLB_RECORDS][1024] EXT_RAM_BSS_ATTR;
static bool  s_rec_charge[CLB_RECORDS];
static int   s_rec_n;

static uint32_t s_frames, s_tx_errors;
static volatile double s_pair_until; /* mono s, 0 = no timed window */
static volatile bool   s_hold_now;

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
        clb_put(&s_core, name, value);
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
    if (s_rec_n == CLB_RECORDS) /* full: drop the oldest */
    {
        memmove(s_rec[0], s_rec[1], sizeof(s_rec[0]) * (CLB_RECORDS - 1));
        memmove(&s_rec_charge[0], &s_rec_charge[1],
                sizeof(s_rec_charge[0]) * (CLB_RECORDS - 1));
        s_rec_n--;
    }
    strlcpy(s_rec[s_rec_n], line, sizeof(s_rec[0]));
    s_rec_charge[s_rec_n] = charge;
    s_rec_n++;
    xSemaphoreGive(s_mx);
}

static bool send(const char *buf, size_t n)
{
    if (n == 0)
    {
        return true;
    }
    int w = ble_manager_channel_write(s_ch, (const uint8_t *)buf, n);
    if (w != (int)n)
    {
        s_tx_errors++;
        return false;
    }
    s_frames++;
    return true;
}

static void send_hello(char *buf, size_t cap, bool hold)
{
    const esp_app_desc_t *app = esp_app_get_description();
    int w = snprintf(buf, cap,
                     "{\"t\":\"hello\",\"proto\":%d,\"fw\":\"%s\","
                     "\"pack_kwh\":%.3f,\"clock_valid\":%s,\"abrp\":\"%s\"}\n",
                     CLB_PROTO, app->version, CL_PACK_KWH,
                     rtc_manager_time_valid() ? "true" : "false",
                     hold ? "phone" : "wican");
    if (w > 0 && (size_t)w < cap)
    {
        send(buf, (size_t)w);
    }
}

static void send_status(char *buf, size_t cap, bool hold)
{
    int w = snprintf(buf, cap, "{\"t\":\"s\",\"ts\":%.0f,\"abrp\":\"%s\","
                               "\"s\":", wall_s(), hold ? "phone" : "wican");
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
    send(buf, n + 2);
}

static void send_records(char *buf, size_t cap)
{
    for (;;)
    {
        bool charge;
        bool have = false;
        xSemaphoreTake(s_mx, portMAX_DELAY);
        if (s_rec_n > 0)
        {
            charge = s_rec_charge[0];
            int w = snprintf(buf, cap, "{\"t\":\"%s\",\"r\":%s}\n",
                             charge ? "charge" : "trip", s_rec[0]);
            have = w > 0 && (size_t)w < cap;
            memmove(s_rec[0], s_rec[1], sizeof(s_rec[0]) * (CLB_RECORDS - 1));
            memmove(&s_rec_charge[0], &s_rec_charge[1],
                    sizeof(s_rec_charge[0]) * (CLB_RECORDS - 1));
            s_rec_n--;
        }
        else
        {
            xSemaphoreGive(s_mx);
            return;
        }
        xSemaphoreGive(s_mx);
        if (have && !send(buf, strlen(buf)))
        {
            return; /* link trouble: the record is lost to BLE only;
                       it is in the history file */
        }
    }
}

static void task_main(void *arg)
{
    (void)arg;
    static char frame[CLB_FRAME_MAX] EXT_RAM_BSS_ATTR;
    uint8_t rx[128];
    bool subscribed = false;
    bool hold = false;
    double next_values = 0, next_status = 0;

    while (s_run)
    {
        int r = ble_manager_channel_read(s_ch, rx, sizeof(rx), 250);
        double now = mono_s();
        if (r > 0)
        {
            xSemaphoreTake(s_mx, portMAX_DELAY);
            clb_rx(&s_core, rx, (size_t)r, now);
            xSemaphoreGive(s_mx);
        }
        else if (r < 0)
        {
            if (subscribed)
            {
                ESP_LOGI(TAG, "phone link down");
            }
            subscribed = false;
            xSemaphoreTake(s_mx, portMAX_DELAY);
            clb_link_down(&s_core);
            xSemaphoreGive(s_mx);
            vTaskDelay(pdMS_TO_TICKS(250)); /* no link: read returns at once */
        }

        xSemaphoreTake(s_mx, portMAX_DELAY);
        bool h = clb_abrp_hold(&s_core, now);
        xSemaphoreGive(s_mx);
        if (h != hold)
        {
            hold = h;
            data_destinations_set_abrp_hold(hold);
            ESP_LOGI(TAG, "ABRP sender: %s", hold ? "phone" : "WiCAN");
        }

        s_hold_now = hold;
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
            subscribed = false;
            continue;
        }
        if (!subscribed)
        {
            subscribed = true;
            ESP_LOGI(TAG, "phone subscribed");
            xSemaphoreTake(s_mx, portMAX_DELAY);
            clb_mark_all_dirty(&s_core);
            xSemaphoreGive(s_mx);
            send_hello(frame, sizeof(frame), hold);
            next_values = 0;
            next_status = 0;
        }
        if (now >= next_values)
        {
            next_values = now + CLB_VALUES_MS / 1000.0;
            for (int i = 0; i < 8; i++) /* a full table spans a few frames */
            {
                xSemaphoreTake(s_mx, portMAX_DELAY);
                size_t n = clb_values_frame(&s_core, frame, sizeof(frame),
                                            wall_s());
                xSemaphoreGive(s_mx);
                if (n == 0 || !send(frame, n))
                {
                    break;
                }
            }
        }
        if (now >= next_status)
        {
            next_status = now + CLB_STATUS_MS / 1000.0;
            send_status(frame, sizeof(frame), hold);
        }
        send_records(frame, sizeof(frame));
    }
    data_destinations_set_abrp_hold(false);
    s_task = NULL;
    vTaskDelete(NULL);
}

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
    if (s_mx != NULL && xSemaphoreTake(s_mx, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        strlcpy(out->app, s_core.app, sizeof(out->app));
        xSemaphoreGive(s_mx);
    }
}

esp_err_t cherylink_ble_start(void)
{
    if (s_task != NULL)
    {
        return ESP_OK;
    }
    if (s_mx == NULL)
    {
        s_mx = xSemaphoreCreateMutex();
        if (s_mx == NULL)
        {
            return ESP_ERR_NO_MEM;
        }
        clb_init(&s_core);
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
            return ESP_OK; /* log-and-degrade: page + WiCAN ABRP still work */
        }
    }
    s_run = true;
    s_task = xTaskCreateStatic(task_main, "cl_ble",
                               sizeof(s_stack) / sizeof(s_stack[0]), NULL, 2,
                               s_stack, &s_tcb);
    if (s_task == NULL)
    {
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "started (channel %d, FFF7/FFF8)", s_ch);
    return ESP_OK;
}

esp_err_t cherylink_ble_stop(void)
{
    s_run = false;
    for (int i = 0; i < 30 && s_task != NULL; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return ESP_OK;
}
