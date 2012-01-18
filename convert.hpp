// Copyright 2011 Ethan Eade. All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:

//    1. Redistributions of source code must retain the above
//       copyright notice, this list of conditions and the following
//       disclaimer.

//    2. Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials
//       provided with the distribution.

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

// The views and conclusions contained in the software and
// documentation are those of the authors and should not be
// interpreted as representing official policies, either expressed or
// implied, of Ethan Eade.

#ifndef ECV_CONVERT_HPP
#define ECV_CONVERT_HPP

#include <ecv/image.hpp>
#include <stdint.h>
#include <ecv/rgb.hpp>

namespace ecv {

    template <class Src, class Dst>
    struct Converter;

    template <class A>
    struct Converter<A,A> {
        static void convert(const A& in, A& out) { out = in; }
    };
    
    template <>
    struct Converter<uint8_t,uint16_t> {
        static void convert(uint8_t in, uint16_t& out) {
            out = 257 * in;
        }
    };

    template <>
    struct Converter<uint16_t,uint8_t> {
        static void convert(uint16_t in, uint8_t& out) {
            out = (in * 255) >> 16;
        }
    };

    template <>
    struct Converter<uint8_t,int16_t> {
        static void convert(uint8_t in, int16_t& out) {
            out = (257*in + 1) >> 1;
        }
    };

    template <>
    struct Converter<int16_t,uint8_t> {
        static void convert(int16_t in, uint8_t& out) {
            out = (in * 255) >> 15;
        }
    };


    template <>
    struct Converter<uint8_t,float> {
        static void convert(uint8_t in, float& out) {
            out = (1.0f/255) * in;
        }
    };

    template <>
    struct Converter<float,uint8_t> {
        static void convert(float in, uint8_t& out) {
            out = (uint8_t)(in * 255 + 0.5f);
        }
    };

    template <>
    struct Converter<int16_t,float> {
        static void convert(int16_t in, float& out) {
            out = (1.0f/32767) * in;
        }
    };

    template <>
    struct Converter<float,int16_t> {
        static void convert(float in, uint16_t& out) {
            out = (uint8_t)(in * 32767 + 0.5f);
        }
    };

    template <>
    struct Converter<uint16_t,float> {
        static void convert(uint16_t in, float& out) {
            out = (1.0f/65535) * in;
        }
    };

    template <>
    struct Converter<float,uint16_t> {
        static void convert(float in, uint16_t& out) {
            out = (uint8_t)(in * 65535 + 0.5f);
        }
    };

    template <>
    struct Converter<Rgb8,uint8_t> {
        static void convert(const Rgb8& in, uint8_t& out) {
            out = (19595*in.r + 38470*in.g + 7471*in.b + 32768) >> 16;
        }
    };    

    template <class Src>
    struct Converter<Src,Rgb8> {
        static void convert(const Src& in, Rgb8& out) {
            Converter<Src,uint8_t>::convert(in, out.r);
            out.g = out.r;
            out.b = out.r;
        }
    };    

    template <>
    struct Converter<Rgb8,uint16_t> {
        static void convert(const Rgb8& in, uint16_t& out) {
            out = (19595*in.r + 38470*in.g + 7471*in.b + 128) >> 8;
        }
    };

    template <>
    struct Converter<Rgb8,float> {
        static void convert(const Rgb8& in, float& out) {
            out = 0.001172549f*in.r + 0.002301961f*in.g + 0.000447059f*in.b;
        }
    };    
    
    template <class Dst, class Src>
    void convert(const Src& src, Dst& dst)
    {
        Converter<Src,Dst>::convert(src, dst);
    }

    template <class Src, class Dst>    
    void convert(const Image<Src>& in, Image<Dst>& out)
    {
        out.resize(in.size());
        for (int i=0; i<out.height(); ++i) {
            const Src *pin = in[i];
            Dst *pout = out[i];
            for (int j=0; j<out.width(); ++j) {
                Converter<Src,Dst>::convert(pin[j], pout[j]);                
            }
        }
    }

    template <class Dst, class Src>    
    Image<Dst> convert(const Image<Src>& in)
    {
        Image<Dst> out(in.size());
        convert(in, out);
        return out;
    }
    
}

#endif
