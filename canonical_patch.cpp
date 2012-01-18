// Copyright 2011 Ethan Eade. All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 
//    1. Redistributions of source code must retain the above
//       copyright notice, this list of conditions and the following
//       disclaimer.
// 
//    2. Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials
//       provided with the distribution.
// 
// THIS SOFTWARE IS PROVIDED BY ETHAN EADE ``AS IS'' AND ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL ETHAN EADE OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
// OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
// USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
// 
// The views and conclusions contained in the software and
// documentation are those of the authors and should not be
// interpreted as representing official policies, either expressed or
// implied, of Ethan Eade.
#include <ecv/canonical_patch.hpp>

using namespace ecv;

Image<float> ecv::make_disc(int size)
{
    Image<float> disc(size,size);
    float c = (size-1)*0.5f;
    float inner_R = c*c;
    float outer_R = (c+1)*(c+1);

    for (int i=0; i<size; ++i)
    {
        float dy2 = (i-c) * (i-c);
        float *o = disc[i];
        for (int j=0; j<size; ++j) {
            float r2 = dy2 + (j-c) * (j-c);
            if (r2 > outer_R) {
                o[j] = 0;
            } else if (r2 < inner_R) {
                o[j] = 1;
            } else {
                float t = sqrtf(r2) - c;
                o[j] = 1 - t;
            } 
        }
    }
    return disc;
}

Image<float> ecv::make_gaussian_envelope(int size, float sigmas_at_size)
{
    Image<float> g(size,size);
    float c = (size-1)*0.5f;
    float factor = -0.5 * latl::sq(sigmas_at_size / c);    

    for (int i=0; i<size; ++i)
    {
        float dy2 = (i-c) * (i-c);
        float *o = g[i];
        for (int j=0; j<size; ++j) {
            float r2 = dy2 + (j-c) * (j-c);
            o[j] = expf(r2 * factor);
        }
    }
    return g;
}
