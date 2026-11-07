/* host test for cl_ble_core: cc -I../include ../cl_ble_core.c test_ble.c -lm */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "cl_ble_core.h"

static void rx(clb_sub_t *c, const char *s, double now)
{
    clb_rx(c, (const uint8_t *)s, strlen(s), now);
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    static clb_t c;
    static clb_sub_t c2, sb;
    static char buf[2048];
    clb_init(&c);
    clb_sub_init(&c2);
    clb_sub_init(&sb);

    assert(clb_values_frame(&c, &c2, buf, sizeof(buf), 1) == 0);
    assert(clb_put(&c, "SOC", 81.25));
    assert(clb_put(&c, "HV_W", -3.5));
    assert(clb_put(&c, "gps_latitude", -26.123456));
    assert(clb_put(&c, "BAD", NAN));
    assert(!clb_put(&c, "bad\"name", 1));
    assert(!clb_put(&c, "", 1));
    assert(!clb_put(&c, "A_NAME_THAT_IS_FAR_TOO_LONG_FOR_IT", 1));
    size_t n = clb_values_frame(&c, &c2, buf, sizeof(buf), 1793734500);
    printf("%s", buf);
    assert(n == strlen(buf));
    assert(strcmp(buf, "{\"t\":\"v\",\"ts\":1793734500,\"v\":{\"SOC\":81.25,"
                       "\"HV_W\":-3.5,\"gps_latitude\":-26.123456,"
                       "\"BAD\":null}}\n") == 0);
    /* nothing changed */
    assert(clb_values_frame(&c, &c2, buf, sizeof(buf), 2) == 0);
    clb_put(&c, "SOC", 81.0);
    n = clb_values_frame(&c, &c2, buf, sizeof(buf), 3);
    assert(strstr(buf, "\"SOC\":81}") != NULL && strstr(buf, "HV_W") == NULL);

    /* table full + small buffer: spills into several frames, all sent */
    for (int i = 0; i < CLB_MAX_VALS + 5; i++)
    {
        char nm[16];
        snprintf(nm, sizeof(nm), "P%03d", i);
        clb_put(&c, nm, i * 1.5);
    }
    assert(c.n == CLB_MAX_VALS && c.skipped >= 5);
    clb_sub_resync(&c2);
    size_t frames = 0, total = 0;
    char small[200];
    while ((n = clb_values_frame(&c, &c2, small, sizeof(small), 4)) > 0)
    {
        assert(n < sizeof(small) && small[n - 1] == '\n' && small[n] == '\0');
        assert(small[n - 2] == '}' && small[n - 3] == '}');
        for (char *p = small; (p = strstr(p, "\":")) != NULL; p++)
        {
            total++;
        }
        frames++;
        assert(frames < 100);
    }
    printf("%zu values in %zu frames of <=200 B\n", total, frames);
    total -= 3 * frames; /* "t":, "ts":, "v": */
    assert(total == CLB_MAX_VALS && frames > 1);

    /* app messages, split across writes, CRLF, overflow */
    assert(!clb_abrp_hold(&c2, 100));
    rx(&c2, "{\"t\":\"hel", 100);
    rx(&c2, "lo\",\"app\":\"ios\",\"proto\":1,\"abrp\":true}\r\n", 100);
    assert(clb_abrp_hold(&c2, 100) && strcmp(c2.app, "ios") == 0);
    assert(clb_abrp_hold(&c2, 159));
    assert(!clb_abrp_hold(&c2, 161));             /* phone gone 60 s  */
    rx(&c2, "{\"t\":\"hb\",\"abrp\":true}\n", 170);
    assert(clb_abrp_hold(&c2, 171));
    rx(&c2, "{\"t\":\"hb\",\"abrp\":false}\n", 175);
    assert(!clb_abrp_hold(&c2, 176));             /* app stopped ABRP */
    rx(&c2, "{\"t\":\"hb\",\"abrp\":true}\n", 180);
    rx(&c2, "{\"t\":\"bye\"}\n", 181);
    assert(!clb_abrp_hold(&c2, 182));
    char big[600];
    memset(big, 'x', sizeof(big));
    clb_rx(&c2, (const uint8_t *)big, sizeof(big), 190);
    rx(&c2, "\"t\":\"hb\",\"abrp\":true}\n", 190);  /* tail of the long line */
    assert(!clb_abrp_hold(&c2, 191));
    rx(&c2, "{\"t\":\"hb\",\"abrp\":true}\n", 192);
    assert(clb_abrp_hold(&c2, 193));
    clb_link_down(&c2);
    assert(clb_abrp_hold(&c2, 200) && !clb_abrp_hold(&c2, 253));
    /* a second subscriber (Wi-Fi) gets everything on its own */
    {
        size_t got = 0, fr = 0;
        char sm[400];
        size_t k;
        while ((k = clb_values_frame(&c, &sb, sm, sizeof(sm), 5)) > 0)
        {
            for (char *p = sm; (p = strstr(p, "\":")) != NULL; p++)
            {
                got++;
            }
            fr++;
        }
        got -= 3 * fr;
        printf("second subscriber: %zu values\n", got);
        assert(got == CLB_MAX_VALS);
        assert(clb_values_frame(&c, &c2, sm, sizeof(sm), 6) == 0);
        clb_put(&c, "SOC", 50);
        assert(clb_values_frame(&c, &sb, sm, sizeof(sm), 7) > 0);
        assert(clb_values_frame(&c, &c2, sm, sizeof(sm), 7) > 0);
        assert(strstr(sm, "\"SOC\":50") != NULL);
        assert(!clb_abrp_hold(&sb, 200));
    }
    printf("OK\n");
    return 0;
}
