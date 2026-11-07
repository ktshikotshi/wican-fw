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
            c->vals[i].dirty = true;
            return true;
        }
    }
    if (c->n >= CLB_MAX_VALS)
    {
        c->skipped++;
        return false;
    }
    clb_val_t *e = &c->vals[c->n++];
    strcpy(e->name, name); /* name_ok: shorter than CLB_NAME_MAX */
    e->v = v;
    e->dirty = true;
    return true;
}

void clb_mark_all_dirty(clb_t *c)
{
    for (size_t i = 0; i < c->n; i++)
    {
        c->vals[i].dirty = true;
    }
}

size_t clb_values_frame(clb_t *c, char *buf, size_t cap, double ts)
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
        clb_val_t *e = &c->vals[i];
        if (!e->dirty)
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
            e->dirty = false; /* cannot happen with name_ok; never loop */
            continue;
        }
        if (n + (size_t)w + tail > cap)
        {
            break; /* stays dirty: the next frame takes it */
        }
        memcpy(buf + n, one, (size_t)w);
        n += (size_t)w;
        e->dirty = false;
        items++;
    }
    if (items == 0)
    {
        return 0;
    }
    memcpy(buf + n, "}}\n", 4); /* incl. NUL */
    return n + 3;
}

static void on_line(clb_t *c, const char *l, double now)
{
    if (strstr(l, "\"t\":\"hello\"") != NULL ||
        strstr(l, "\"t\":\"hb\"") != NULL)
    {
        c->last_msg = now;
        c->phone_abrp = strstr(l, "\"abrp\":true") != NULL;
        c->msgs++;
        const char *a = strstr(l, "\"app\":\"");
        if (a != NULL)
        {
            a += 7;
            size_t k = 0;
            while (k + 1 < sizeof(c->app) &&
                   ((a[k] >= 'a' && a[k] <= 'z') ||
                    (a[k] >= 'A' && a[k] <= 'Z') ||
                    (a[k] >= '0' && a[k] <= '9') || a[k] == '-' ||
                    a[k] == '_' || a[k] == '.'))
            {
                c->app[k] = a[k];
                k++;
            }
            c->app[k] = '\0';
        }
    }
    else if (strstr(l, "\"t\":\"bye\"") != NULL)
    {
        c->phone_abrp = false;
        c->msgs++;
    }
}

void clb_rx(clb_t *c, const uint8_t *d, size_t n, double now)
{
    for (size_t i = 0; i < n; i++)
    {
        char ch = (char)d[i];
        if (ch == '\n' || ch == '\r')
        {
            if (!c->line_overflow && c->line_n > 0)
            {
                c->line[c->line_n] = '\0';
                on_line(c, c->line, now);
            }
            c->line_n = 0;
            c->line_overflow = false;
            continue;
        }
        if (c->line_n + 1 >= sizeof(c->line))
        {
            c->line_overflow = true; /* drop the rest of this line */
            continue;
        }
        c->line[c->line_n++] = ch;
    }
}

void clb_link_down(clb_t *c)
{
    c->line_n = 0;
    c->line_overflow = false;
}

bool clb_abrp_hold(const clb_t *c, double now)
{
    return c->phone_abrp && c->last_msg > 0 && now - c->last_msg < CLB_HOLD_S;
}
