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

/* decoded coordinates */
struct dec_location {
    float lat;
    float lng;
};

/* encoded coordinates */
struct enc_location {
    int lat;
    int lng;
};

/* CPR decoding functions.
 * Decoding depends on type 0 for even messages and type 1 for odd.
 * Surface == 0 for airborne positions, 1 for surface */

int nz(int type);
float nl(float lat);
float dlat(int type, int surface);
float dlng(float dec_lat, int type, int surface);
float decode_lat(int enc_lat, int type, int surface, float recv_lat);
float decode_lng(float dec_lat, int enc_lon, int type, int surface, float recv_lng);
struct dec_location *cpr_resolve_local(struct dec_location *recv_loc,
        struct enc_location *loc, int type, int surface);
