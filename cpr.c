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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include <fcntl.h>
#include <ctype.h>

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
    float res = (surface != 0) ? 90.0 : 360.0;
    int nz_val = nz(type);
    if (nz_val != 0)
        return res / nz_val;
    else
        return res;
}

float dlng(float dec_lat, int type, int surface) {
    float res = (surface != 0) ? 90.0 : 360.0;
    int nl_val = MAX(nl(dec_lat) - type, 1);
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
