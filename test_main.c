/* Host unit tests for the hardware-independent core (FR-20). Build: make -C test/host */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "wt_config.h"
#include "wt_crc8.h"
#include "wt_payload.h"
#include "wt_signal.h"
#include "wt_calib.h"
#include "wt_step.h"
#include "wt_fall.h"
#include "wt_posture.h"
#include "wt_activity.h"

static int fails = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("  FAIL line %d: %s\n", __LINE__, #c); } } while (0)

static void t_crc_payload(void)
{
    printf("crc/payload\n");
    CHECK(wt_crc8((const uint8_t *)"123456789", 9) == 0xF4);
    wt_status_t s = { .flags = WT_FLAG_FALL | WT_FLAG_WALKING, .state = 2, .steps = 0x01020304,
                      .tilt_deg = 200, .falls = 7, .uptime_min = 0x1234, .violations = 3 };
    uint8_t p[WT_PAYLOAD_LEN];
    wt_payload_encode(&s, 9, p);
    CHECK(p[0] == 1 && p[1] == 9 && p[2] == 0x05 && p[3] == 2);
    CHECK(p[4] == 1 && p[5] == 2 && p[6] == 3 && p[7] == 4);   /* big-endian steps */
    CHECK(p[8] == 180 && p[9] == 7 && p[10] == 0x12 && p[11] == 0x34 && p[12] == 3);
    CHECK(p[13] == wt_crc8(p, 13));
    char name[6]; wt_name_from_mac_byte(0xAb, name);
    CHECK(strcmp(name, "WT_AB") == 0);
    uint8_t adv[31]; size_t n = wt_adv_build(name, p, adv, sizeof adv);
    CHECK(n == 28);
    CHECK(adv[0] == 2 && adv[1] == 1 && adv[3] == 6 && adv[4] == 9 && memcmp(&adv[5], "WT_AB", 5) == 0);
    CHECK(adv[10] == 17 && adv[11] == 0xFF && adv[12] == 0xFF && adv[13] == 0xFF);
    CHECK(memcmp(&adv[14], p, 14) == 0);
    CHECK(wt_adv_build("THIS_NAME_IS_FAR_TOO_LONG", p, adv, sizeof adv) == 0);
}

static void t_tilt(void)
{
    printf("tilt\n");
    wt_vec3_t up = {0, 0, 1}, side = {1, 0, 0}, inv = {0, 0, -1}, t30 = {0.5f, 0, 0.8660254f};
    CHECK(fabsf(wt_tilt_deg(&up, &up)) < 0.01f);
    CHECK(fabsf(wt_tilt_deg(&side, &up) - 90) < 0.01f);
    CHECK(fabsf(wt_tilt_deg(&inv, &up) - 180) < 0.01f);
    CHECK(fabsf(wt_tilt_deg(&t30, &up) - 30) < 0.01f);
}

static void t_calib(void)
{
    printf("calib\n");
    wt_calib_t c; wt_calib_start(&c);
    uint32_t t = 0; int restarted = 0; wt_calib_status_t st = WT_CALIB_RUNNING;
    /* 1.5 s still, then a bump, then 3 s still: must restart once and finish ~3 s after bump */
    for (; t < 1500; t += 20) st = wt_calib_feed(&c, &(wt_vec3_t){0.02f, 0.0f, 1.0f}, t);
    CHECK(st == WT_CALIB_RUNNING);
    st = wt_calib_feed(&c, &(wt_vec3_t){0.6f, 0.0f, 1.0f}, t); restarted += (st == WT_CALIB_RESTARTED); t += 20;
    uint32_t t_after = t;
    for (; t < t_after + 3100 && st != WT_CALIB_DONE; t += 20)
        st = wt_calib_feed(&c, &(wt_vec3_t){0.0f, 0.02f, 1.0f}, t);
    CHECK(restarted == 1 && st == WT_CALIB_DONE);
    CHECK(fabsf(c.ref.z - 1.0f) < 0.01f);
    CHECK(t - t_after >= CALIB_DURATION_MS);
}

static int run_walk(wt_step_t *s, uint32_t *t, float hz, float amp, int seconds, int *credited_events)
{
    int total = 0;
    for (int i = 0; i < seconds * ODR_NORMAL_HZ; i++, *t += 20) {
        float m = 1.0f + amp * sinf(2 * 3.14159265f * hz * (*t / 1000.0f));
        int c = wt_step_feed(s, m, *t);
        total += c; if (c && credited_events) (*credited_events)++;
    }
    return total;
}

static void t_steps(void)
{
    printf("steps\n");
    wt_step_t s; wt_step_init(&s, ODR_NORMAL_HZ);
    uint32_t t = 0;
    /* quiet then walk 20 steps at 2 Hz (10 s) */
    run_walk(&s, &t, 2.0f, 0.0f, 3, NULL);
    int n = run_walk(&s, &t, 2.0f, 0.35f, 10, NULL);
    printf("  walked 20 steps -> counted %d\n", n);
    CHECK(n >= 18 && n <= 22);
    CHECK(s.steps == (uint32_t)n);

    /* isolated single bump and a pair of bumps must not count (FR-5) */
    wt_step_t b; wt_step_init(&b, ODR_NORMAL_HZ); t = 0;
    int got = 0;
    for (int i = 0; i < 150; i++, t += 20) {
        float m = 1.0f;
        if (i >= 60 && i < 66) m += 0.8f * sinf(3.14159265f * (i - 60) / 6.0f);   /* bump 1 */
        if (i >= 100 && i < 106) m += 0.8f * sinf(3.14159265f * (i - 100) / 6.0f);/* bump 2, 800 ms later */
        got += wt_step_feed(&b, m, t);
    }
    CHECK(got == 0);

    /* three bumps 600 ms apart -> exactly 3 credited retroactively at once */
    wt_step_t c; wt_step_init(&c, ODR_NORMAL_HZ); t = 0; int events = 0; got = 0;
    for (int i = 0; i < 250; i++, t += 20) {
        float m = 1.0f;
        for (int k = 0; k < 3; k++) { int s0 = 60 + k * 30; if (i >= s0 && i < s0 + 6) m += 0.8f * sinf(3.14159265f * (i - s0) / 6.0f); }
        int r = wt_step_feed(&c, m, t); got += r; if (r) { events++; CHECK(r == 3); }
    }
    CHECK(got == 3 && events == 1);

    /* random hand shaking 5 s: at most 3 steps (AC-2). 50 different seeds. */
    int worst = 0;
    for (int seed = 1; seed <= 50; seed++) {
        srand(seed);
        wt_step_t sh; wt_step_init(&sh, ODR_NORMAL_HZ); t = 0; got = 0;
        for (int i = 0; i < 100; i++) wt_step_feed(&sh, 1.0f, t += 20);
        for (int i = 0; i < 250; i++, t += 20) {
            float m = 1.0f + 1.2f * (((rand() % 2000) / 1000.0f) - 1.0f);
            got += wt_step_feed(&sh, m, t);
        }
        if (got > worst) worst = got;
    }
    printf("  shake 5 s x50 seeds -> worst case %d steps\n", worst);
    CHECK(worst <= 3);
}

static uint8_t feed_seq(wt_fall_t *f, const wt_vec3_t *v, uint32_t *t, int dur_ms, int dt_ms,
                        const wt_vec3_t *ref, uint32_t *t_confirm)
{
    uint8_t all = 0;
    for (int e = 0; e < dur_ms; e += dt_ms, *t += dt_ms) {
        uint8_t ev = wt_fall_feed(f, v, *t, ref);
        all |= ev;
        if ((ev & WT_FALL_EV_STAGE3) && t_confirm) *t_confirm = *t;
    }
    return all;
}

static void t_fall(void)
{
    printf("fall\n");
    wt_vec3_t up = {0, 0, 1}, ff = {0, 0, 0.1f}, hit = {0, 0, 3.5f}, ring = {0, 0, 1.6f};
    wt_vec3_t lying = {1.0f, 0.0f, 0.0f};
    wt_fall_t f; wt_fall_init(&f);
    uint32_t t = 0, tc = 0, t_imp;

    feed_seq(&f, &up, &t, 1000, 5, &up, NULL);
    uint8_t ev = feed_seq(&f, &ff, &t, 245, 5, &up, NULL);
    CHECK(ev & WT_FALL_EV_STAGE1);
    t_imp = t; ev = feed_seq(&f, &hit, &t, 5, 5, &up, NULL);
    CHECK(ev & WT_FALL_EV_STAGE2);
    ev = feed_seq(&f, &ring, &t, 40, 5, &up, NULL);
    ev |= feed_seq(&f, &lying, &t, 2500, 5, &up, &tc);
    CHECK(ev & WT_FALL_EV_STAGE3);
    printf("  impact->confirm = %u ms, tilt=%.1f deg, inactivity=%u ms\n",
           (unsigned)(tc - t_imp), f.last_tilt, (unsigned)f.last_inactivity_ms);
    CHECK(tc - t_imp < 2300 && tc - t_imp >= INACTIVITY_MS);
    CHECK(f.last_inactivity_ms >= INACTIVITY_MS - 100 && f.last_inactivity_ms <= INACTIVITY_MS + 100);

    /* same sequence but board ends upright -> tilt gate rejects (FR-7 stage 3) */
    wt_fall_init(&f); t = 0;
    feed_seq(&f, &ff, &t, 245, 5, &up, NULL);
    feed_seq(&f, &hit, &t, 5, 5, &up, NULL);
    ev = feed_seq(&f, &up, &t, 2500, 5, &up, NULL);
    CHECK(!(ev & WT_FALL_EV_STAGE3) && (ev & WT_FALL_EV_DISCARDED) && f.last_discard == WT_FALL_DISCARD_TILT);

    /* user moves after impact -> discard, no alert (FR-10) */
    wt_fall_init(&f); t = 0;
    feed_seq(&f, &ff, &t, 245, 5, &up, NULL);
    feed_seq(&f, &hit, &t, 5, 5, &up, NULL);
    feed_seq(&f, &lying, &t, 500, 5, &up, NULL);
    ev = feed_seq(&f, &(wt_vec3_t){0, 0, 1.6f}, &t, 100, 5, &up, NULL);
    CHECK((ev & WT_FALL_EV_DISCARDED) && f.last_discard == WT_FALL_DISCARD_MOVED && !wt_fall_active(&f));

    /* free-fall without impact -> timeout discard */
    wt_fall_init(&f); t = 0;
    feed_seq(&f, &ff, &t, 100, 5, &up, NULL);
    ev = feed_seq(&f, &up, &t, 1200, 5, &up, NULL);
    CHECK((ev & WT_FALL_EV_DISCARDED) && f.last_discard == WT_FALL_DISCARD_IMPACT_TIMEOUT);

    /* too-short free-fall (40 ms) -> never a candidate beyond stage 1 */
    wt_fall_init(&f); t = 0;
    ev = feed_seq(&f, &ff, &t, 40, 5, &up, NULL);
    ev |= feed_seq(&f, &hit, &t, 50, 5, &up, NULL);
    CHECK(ev == 0 && !wt_fall_active(&f));

    /* 50 Hz sampling still detects a 245 ms drop */
    wt_fall_init(&f); t = 0; tc = 0;
    feed_seq(&f, &up, &t, 1000, 20, &up, NULL);
    feed_seq(&f, &ff, &t, 240, 20, &up, NULL);
    feed_seq(&f, &hit, &t, 20, 20, &up, NULL);
    ev = feed_seq(&f, &lying, &t, 2600, 20, &up, &tc);
    CHECK(ev & WT_FALL_EV_STAGE3);
}

static void t_posture(void)
{
    printf("posture\n");
    wt_posture_t p; wt_posture_init(&p, ODR_NORMAL_HZ);
    wt_vec3_t ref = {0, 0, 1};
    wt_vec3_t up = {0, 0, 1}, t30 = {0.5f, 0, 0.8660254f};
    uint32_t t = 0, t_on = 0, t_off = 0, t_start;
    for (int i = 0; i < 100; i++, t += 20) wt_posture_feed(&p, &up, t, &ref);
    t_start = t;
    for (int i = 0; i < 40 * 50; i++, t += 20)
        if (wt_posture_feed(&p, &t30, t, &ref) == WT_POSTURE_EV_ON) t_on = t;
    printf("  posture ON after %u ms of 30 deg\n", (unsigned)(t_on - t_start));
    CHECK(t_on && t_on - t_start >= 29000 && t_on - t_start <= 31000);
    t_start = t;
    for (int i = 0; i < 8 * 50; i++, t += 20)
        if (wt_posture_feed(&p, &up, t, &ref) == WT_POSTURE_EV_OFF) t_off = t;
    printf("  posture OFF after %u ms upright\n", (unsigned)(t_off - t_start));
    CHECK(t_off && t_off - t_start >= 4000 && t_off - t_start <= 6500);

    /* hysteresis: hovering at 22 deg while bad must not clear */
    wt_posture_init(&p, ODR_NORMAL_HZ); t = 0;
    wt_vec3_t t22 = {sinf(22 * 3.14159265f / 180), 0, cosf(22 * 3.14159265f / 180)};
    wt_vec3_t t26 = {sinf(26 * 3.14159265f / 180), 0, cosf(26 * 3.14159265f / 180)};
    int ons = 0, offs = 0;
    for (int i = 0; i < 35 * 50; i++, t += 20) ons += wt_posture_feed(&p, &t26, t, &ref) == WT_POSTURE_EV_ON;
    for (int i = 0; i < 20 * 50; i++, t += 20) offs += wt_posture_feed(&p, &t22, t, &ref) == WT_POSTURE_EV_OFF;
    CHECK(ons == 1 && offs == 0 && p.bad);
}

static void t_activity(void)
{
    printf("activity\n");
    wt_activity_t a; wt_act_init(&a);
    CHECK(a.state == WT_STATE_CALIBRATING);
    wt_act_calibration_done(&a); CHECK(a.state == WT_STATE_IDLE);
    wt_act_on_step(&a, 1000); CHECK(a.state == WT_STATE_WALKING && a.walking);
    CHECK(!wt_act_tick(&a, 3000)); CHECK(wt_act_tick(&a, 4000) && a.state == WT_STATE_IDLE);
    wt_act_on_fall(&a, 10000);
    CHECK(a.state == WT_STATE_ALERT && a.fall_flag && a.falls == 1);
    CHECK(wt_act_fast_alert(&a, 39000) && !wt_act_fast_alert(&a, 40001));
    CHECK(!wt_act_cancel(&a, 20001));                      /* outside 10 s window */
    CHECK(a.fall_flag);
    wt_act_on_fall(&a, 50000);
    CHECK(wt_act_cancel(&a, 55000) && !a.fall_flag && a.state == WT_STATE_IDLE && a.falls == 2);
    wt_act_on_fall(&a, 60000); wt_act_begin_calibration(&a);
    CHECK(!a.fall_flag && a.state == WT_STATE_CALIBRATING);
    for (int i = 0; i < 300; i++) a.falls++;               /* 8-bit wrap */
    CHECK(a.falls == (uint8_t)(3 + 300));
}

int main(void)
{
    t_crc_payload(); t_tilt(); t_calib(); t_steps(); t_fall(); t_posture(); t_activity();
    printf("\n%d checks, %d failed\n", checks, fails);
    return fails ? 1 : 0;
}
