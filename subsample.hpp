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
#ifndef ECV_SUBSAMPLE_HPP
#define ECV_SUBSAMPLE_HPP

#include <ecv/image.hpp>
#include <stdint.h>

namespace ecv {

    inline
    void bin_and_subsample_by_two(const Image<uint8_t>& im,
                                  Image<uint8_t> &half)
    {
        half.resize(im.width()/2, im.height()/2);
        const int s = im.stride();
        for (int i=0; i<half.height(); ++i) {
            const uint8_t *p = im[i*2];
            uint8_t* o = half[i];
            for (int j=0; j<half.width(); ++j) {
                *o = (p[0] + p[1] + p[s] + p[s+1] + 2) >> 2;
                ++o;
                p += 2;
            }
        }
    }
    
    template <class T>
    void subsample_three_fourths_horizontal(const Image<T>& im,
                                            Image<T>& smaller)
    {
        const float third = 1.0f/3;
        const float two_thirds = 1 - third;
        const int w = (im.width()*3 + 1) / 4;
        
        smaller.resize(w, im.height());

        for (int i=0; i<im.height(); ++i) {
            const T* p = im[i];
            T* o = smaller[i];
            
            int j=0;
            for (; j+2<w; j+=3, p+=4) {
                o[j] = p[0];
                o[j+1] = (T)(two_thirds*p[1] + third*p[2]);
                o[j+2] = (T)(third*p[2] + two_thirds*p[3]);
            }

            switch (w-j) {
            case 2: o[j+1] = (T)(two_thirds*p[1]+third*p[2]);
            case 1: o[j] = p[0];
            case 0: break;
            }
        }
    }

    template <class T>
    void subsample_three_fourths_vertical(const Image<T>& im,
                                          Image<T>& smaller)
    {
        const float third = 1.0f/3;
        const float two_thirds = 1 - third;
        const int h = (im.height()*3 + 1) / 4;

        smaller.resize(im.width(), h);

        const T* p = im[0];
        T* o = smaller[0];

        for (int i=0, k=0; i<h; ++i) {
            const T *n = p + im.stride();
            switch (k) {
            case 0:
                for (int j=0; j<im.width(); ++j)
                    o[j] = p[j];
                k = 1;
                break;
            case 1:
                for (int j=0; j<im.width(); ++j)
                    o[j] = (T)(two_thirds*p[j] + third*n[j]);
                k = 2;
                break;
            case 2:
                for (int j=0; j<im.width(); ++j)
                    o[j] = (T)(third*p[j] + two_thirds*n[j]);
                p += im.stride();
                k = 0;
                break;
            }
            p += im.stride();
            o += smaller.stride();
        }
    }

    template <class T>
    void subsample_three_fourths(const Image<T>& im,
                                 Image<T>& smaller)
    {
        Image<T> tmp;
        subsample_three_fourths_horizontal(im, tmp);
        subsample_three_fourths_vertical(tmp, smaller);
    }


    void subsample_two_thirds_bicubic(const Image<uint8_t>& im, Image<uint8_t> &smaller);
    void subsample_four_fifths_bicubic(const Image<uint8_t>& im, Image<uint8_t> &smaller);

    
}

#endif
