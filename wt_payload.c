#include "wt_payload.h"
#include "wt_crc8.h"
#include <string.h>

uint8_t wt_flags_build(int fall, int posture, int walking, int sensor_fault, int deadline_seen)
{
    uint8_t f = 0;
    if (fall)          f |= WT_FLAG_FALL;
    if (posture)       f |= WT_FLAG_POSTURE;
    if (walking)       f |= WT_FLAG_WALKING;
    if (sensor_fault)  f |= WT_FLAG_SENSOR;
    if (deadline_seen) f |= WT_FLAG_DEADLINE;
    return f;
}

void wt_payload_encode(const wt_status_t *s, uint8_t seq, uint8_t out[WT_PAYLOAD_LEN])
{
    out[0]  = WT_PAYLOAD_VER;
    out[1]  = seq;
    out[2]  = s->flags & 0x1F;
    out[3]  = s->state;
    out[4]  = (uint8_t)(s->steps >> 24);
    out[5]  = (uint8_t)(s->steps >> 16);
    out[6]  = (uint8_t)(s->steps >> 8);
    out[7]  = (uint8_t)(s->steps);
    out[8]  = s->tilt_deg > 180 ? 180 : s->tilt_deg;
    out[9]  = s->falls;
    out[10] = (uint8_t)(s->uptime_min >> 8);
    out[11] = (uint8_t)(s->uptime_min);
    out[12] = s->violations;
    out[13] = wt_crc8(out, 13);
}

void wt_name_from_mac_byte(uint8_t b, char out[6])
{
    static const char hex[] = "0123456789ABCDEF";
    out[0] = 'W'; out[1] = 'T'; out[2] = '_';
    out[3] = hex[b >> 4]; out[4] = hex[b & 0x0F]; out[5] = 0;
}

size_t wt_adv_build(const char *name, const uint8_t payload[WT_PAYLOAD_LEN], uint8_t *out, size_t cap)
{
    size_t nl = strlen(name);
    size_t total = 3 + (2 + nl) + (2 + 2 + WT_PAYLOAD_LEN);
    if (total > 31 || total > cap) return 0;
    size_t i = 0;
    out[i++] = 0x02; out[i++] = 0x01; out[i++] = 0x06;            /* Flags: LE general disc, no BR/EDR */
    out[i++] = (uint8_t)(1 + nl); out[i++] = 0x09;                 /* Complete local name */
    memcpy(&out[i], name, nl); i += nl;
    out[i++] = (uint8_t)(1 + 2 + WT_PAYLOAD_LEN); out[i++] = 0xFF; /* Manufacturer specific */
    out[i++] = (uint8_t)(WT_COMPANY_ID & 0xFF);                    /* company ID little-endian */
    out[i++] = (uint8_t)(WT_COMPANY_ID >> 8);
    memcpy(&out[i], payload, WT_PAYLOAD_LEN); i += WT_PAYLOAD_LEN;
    return i;
}
