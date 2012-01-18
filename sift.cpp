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
#include <ecv/sift.hpp>
#include <iostream>

using namespace ecv;
using namespace ecv::sift;

void ecv::sift::build_histogram_entries(int bins, int angles, int patch_size, double sigmas_at_edge,
                                        std::vector<HistogramEntry>& entries)
{
    const int size = patch_size-2;
    entries.resize(size*size);
    const float inv_size = 1.0f/size;
    float center = (size-1) * 0.5f;
    for (int i=0, k=0; i<size; ++i) {
        float dy = 2 * (i - center) * inv_size * sigmas_at_edge;
        float ty = inv_size * (i + 0.5f);
        float biny = bins * ty - 0.5f;

        int ylo = (int)floorf(biny);
        float yfrac = biny - ylo;
                
        for (int j=0; j<size; ++j, ++k) {
            float dx = 2 * (j - center) * inv_size * sigmas_at_edge;
            float r2 = dx*dx + dy*dy;
            float w = expf(-0.5 * r2);

            float tx = inv_size * (j + 0.5f);
            float binx = bins * tx - 0.5f;

            float xlo = (int)floorf(binx);
            float xfrac = binx - xlo;

            HistogramEntry& e = entries[k];

            e.ylo = ylo;
            e.dy = bins*angles;
            e.xlo = xlo;
            e.dx = angles;
                    
            e.weight[0] = (1-yfrac) * (1-xfrac) * w;
            e.weight[1] = (1-yfrac) * xfrac * w;
            e.weight[2] = yfrac * (1-xfrac) * w;
            e.weight[3] = yfrac * xfrac * w;
        }
    }

    for (size_t i=0; i<entries.size(); ++i) {
        HistogramEntry& e = entries[i];
        if (e.ylo < 0) {
            e.ylo = 0;
            e.dy = 0;
            e.weight[0] = e.weight[1] = 0;
        } else if (e.ylo == bins-1) {
            e.dy = 0;
            e.weight[2] = e.weight[3] = 0;
        }
        if (e.xlo < 0) {
            e.xlo = 0;
            e.dx = 0;
            e.weight[0] = e.weight[2] = 0;
        } else if (e.xlo == bins-1) {
            e.dx = 0;
            e.weight[1] = e.weight[3] = 0;
        }
    }

    
}
