/* Compact position reporting (CPR) utility functions
 *
 * Copyright (C) 2012 by Yuval Adam <yuv.adm@gmail.com>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *  *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <math.h>
#include "cpr.h"

#define LATZ 15
#define MAX(x, y) (((x) > (y)) ? (x) : (y))

/* CPR decoding functions.
 * Decoding depends on type 0 for even messages and type 1 for odd.
 * Surface == 0 for airborne positions, 1 for surface */

int nz(int type) {
    return 4 * LATZ - type;
}

float nl(float lat) {
    if (abs(lat) >= 87.0)
        return 1.0;
    return floor(
        (2.0 * M_PI) *
        pow(acos(1.0 - (1.0 - cos(M_PI / (2.0 * LATZ))) /
            pow(cos((M_PI / 180.0) * abs(lat)), 2)), -1)
    );
}

float dlat(int type, int surface) {
    float res = surface ? 90.0 : 360.0;
    int nz_val = nz(type);
    if (nz_val)
        return res / nz_val;
    else
        return res;
}

float dlng(float dec_lat, int type, int surface) {
    float res = surface ? 90.0 : 360.0;
    float nl_val = MAX(nl(dec_lat) - type, 1);
    return res / nl_val;
}

float decode_lat(int enc_lat, int type, int surface, float recv_lat) {
    float t1 = dlat(type, surface);
    float t2 = ((float) enc_lat) / pow(2, 17);
    float j = floor(recv_lat / t1) +
        floor(0.5 + (fmod(recv_lat, t1) / t1) - t2);
    return t1 * (j + t2);
}

float decode_lng(float dec_lat, int enc_lon, int type, int surface, float recv_lng) {
    float t1 = dlng(dec_lat, type, surface);
    float t2 = ((float) enc_lon) / pow(2, 17);
    float m = floor(recv_lng / t1) +
        floor(0.5 + (fmod(recv_lng, t1) / t1) - t2);
    return t1 * (m + t2);
}

struct dec_location *cpr_resolve_local(struct dec_location *recv, 
        struct enc_location *loc, int type, int surface) {
    float dec_lat = decode_lat(loc->lat, type, surface, recv->lat);
    float dec_lng = decode_lng(dec_lat, loc->lng, type, surface, recv->lng);
    struct dec_location *dec_loc = malloc(sizeof(*dec_loc));
    dec_loc->lat = dec_lat;
    dec_loc->lng = dec_lng;
    return dec_loc;
}

struct dec_location *cpr_resolve_global(struct enc_location *even, 
        struct enc_location *odd, struct dec_location *recv, int recent_type, int surface) {
    float dlat_even = dlat(0, surface);
    float dlat_odd = dlat(1, surface);

    float even_lat = (float) even->lat;
    float even_lng = (float) even->lng;
    float odd_lat = (float) odd->lat;
    float odd_lng = (float) odd->lng;

    /* latitude index */
    float j = floor(((nz(1) * even_lat - nz(0) * odd_lat) /
        pow(2,17)) + 0.5);

    float rlat_even = dlat_even * ((fmod(j, nz(0))) + even_lat / pow(2,17));
    float rlat_odd = dlat_odd * ((fmod(j, nz(1))) + odd_lat / pow(2,17));

    if (rlat_even > 270.0)
        rlat_even -= 360.0;
    if (rlat_odd > 270.0)
        rlat_odd -= 360.0;

    if (nl(rlat_even) != nl(rlat_odd))
        return 0;

    float rlat = recent_type ? rlat_odd : rlat_even;

    if (surface && recv->lat < 0)
        rlat -= 90.0;

    float dl = dlng(rlat, recent_type, surface);
    float nl_rlat = nl(rlat);

    /* longitude index */
    float m = floor((((nl_rlat - 1) * even_lng - nl_rlat * odd_lng) / 
        pow(2,17)) + 0.5);

    printf("%f %f\n", dl, nl_rlat);

    float enc_lon = recent_type ? odd_lng : even_lng;

    float rlng = dl * ((fmod(m, MAX(nl_rlat - recent_type, 1))) + enc_lon / pow(2,17));

    if (surface) {
        float wat = recv->lng;
        if (wat < 0)
            wat += 360.0;
        wat = 90 * ((int) wat / 90);
        rlng += (wat - (90 * ((int) rlng / 90)));
    }

    if (rlng > 180) {
        rlng -= 360.0;
    }

    struct dec_location *dec_loc = malloc(sizeof(*dec_loc));
    dec_loc->lat = rlat;
    dec_loc->lng = rlng;
    return dec_loc;
}
