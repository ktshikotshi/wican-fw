/*
 * cl_ble_core: see include/cl_ble_core.h. Pure C, no ESP-IDF.
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include "cl_ble_core.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

void clb_init(clb_t *c)
{
    memset(c, 0, sizeof(*c));
}

static bool name_ok(const char *s)
{
    size_t n = 0;
    for (; s[n] != '\0'; n++)
    {
        unsigned char ch = (unsigned char)s[n];
        if (ch < 0x20 || ch == '"' || ch == '\\' || n + 1 >= CLB_NAME_MAX)
        {
            return false;
        }
    }
    return n > 0;
}

static uint32_t next_seq(clb_t *c)
{
    if (++c->seq == 0) /* 0 means "never sent": skip it on wrap */
    {
        c->seq = 1;
    }
    return c->seq;
}

bool clb_put(clb_t *c, const char *name, double v)
{
    if (name == NULL || !name_ok(name))
    {
        c->skipped++;
        return false;
    }
    for (size_t i = 0; i < c->n; i++)
    {
        if (strcmp(c->vals[i].name, name) == 0)
        {
            c->vals[i].v = v;
            c->vals[i].seq = next_seq(c);
            return true;
        }
    }
    if (c->n >= CLB_MAX_VALS)
    {
        c->skipped++;
        return false;
    }
    clb_val_t *e = &c->vals[c->n];
    strcpy(e->name, name); /* name_ok: shorter than CLB_NAME_MAX */
    e->v = v;
    e->seq = next_seq(c);
    c->n++;
    return true;
}

void clb_sub_init(clb_sub_t *s)
{
    memset(s, 0, sizeof(*s));
}

void clb_sub_resync(clb_sub_t *s)
{
    memset(s->sent, 0, sizeof(s->sent));
}

size_t clb_values_frame(const clb_t *c, clb_sub_t *s, char *buf, size_t cap,
                        double ts)
{
    /* "}}\n" + NUL closes the frame */
    const size_t tail = 4;
    if (cap < 64)
    {
        return 0;
    }
    int w = snprintf(buf, cap, "{\"t\":\"v\",\"ts\":%.0f,\"v\":{", ts);
    if (w < 0 || (size_t)w >= cap)
    {
        return 0;
    }
    size_t n = (size_t)w;
    size_t items = 0;
    for (size_t i = 0; i < c->n; i++)
    {
        const clb_val_t *e = &c->vals[i];
        if (e->seq == s->sent[i])
        {
            continue;
        }
        char one[CLB_NAME_MAX + 40];
        if (isnan(e->v) || isinf(e->v))
        {
            w = snprintf(one, sizeof(one), "%s\"%s\":null", items ? "," : "",
                         e->name);
        }
        else
        {
            w = snprintf(one, sizeof(one), "%s\"%s\":%.9g", items ? "," : "",
                         e->name, e->v);
        }
        if (w < 0 || (size_t)w >= sizeof(one))
        {
            s->sent[i] = e->seq; /* cannot happen with name_ok; never loop */
            continue;
        }
        if (n + (size_t)w + tail > cap)
        {
            break; /* not marked: the next frame takes it */
        }
        memcpy(buf + n, one, (size_t)w);
        n += (size_t)w;
        s->sent[i] = e->seq;
        items++;
    }
    if (items == 0)
    {
        return 0;
    }
    memcpy(buf + n, "}}\n", 4); /* incl. NUL */
    return n + 3;
}

static void on_line(clb_sub_t *s, const char *l, double now)
{
    if (strstr(l, "\"t\":\"hello\"") != NULL ||
        strstr(l, "\"t\":\"hb\"") != NULL)
    {
        s->last_msg = now;
        s->phone_abrp = strstr(l, "\"abrp\":true") != NULL;
        s->msgs++;
        const char *a = strstr(l, "\"app\":\"");
        if (a != NULL)
        {
            a += 7;
            size_t k = 0;
            while (k + 1 < sizeof(s->app) &&
                   ((a[k] >= 'a' && a[k] <= 'z') ||
                    (a[k] >= 'A' && a[k] <= 'Z') ||
                    (a[k] >= '0' && a[k] <= '9') || a[k] == '-' ||
                    a[k] == '_' || a[k] == '.'))
            {
                s->app[k] = a[k];
                k++;
            }
            s->app[k] = '\0';
        }
    }
    else if (strstr(l, "\"t\":\"bye\"") != NULL)
    {
        s->phone_abrp = false;
        s->msgs++;
    }
}

void clb_rx(clb_sub_t *s, const uint8_t *d, size_t n, double now)
{
    for (size_t i = 0; i < n; i++)
    {
        char ch = (char)d[i];
        if (ch == '\n' || ch == '\r')
        {
            if (!s->line_overflow && s->line_n > 0)
            {
                s->line[s->line_n] = '\0';
                on_line(s, s->line, now);
            }
            s->line_n = 0;
            s->line_overflow = false;
            continue;
        }
        if (s->line_n + 1 >= sizeof(s->line))
        {
            s->line_overflow = true; /* drop the rest of this line */
            continue;
        }
        s->line[s->line_n++] = ch;
    }
}

void clb_link_down(clb_sub_t *s)
{
    s->line_n = 0;
    s->line_overflow = false;
}

bool clb_abrp_hold(const clb_sub_t *s, double now)
{
    return s->phone_abrp && s->last_msg > 0 && now - s->last_msg < CLB_HOLD_S;
}
