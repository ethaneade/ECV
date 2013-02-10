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
#ifndef ECV_BRIEF_HPP
#define ECV_BRIEF_HPP

#include "stdint.h"
#include <climits>

template <class T>
unsigned int count_set_bits(T x)
{
    // Courtesy of Sean Eron Anderson
    x = x - ((x >> 1) & (T)~(T)0/3);
    x = (x & (T)~(T)0/15*3) + ((x >> 2) & (T)~(T)0/15*3);
    x = (x + (x >> 4)) & (T)~(T)0/255*15;
    return (unsigned int)((T)(x * ((T)~(T)0/255)) >> (sizeof(T) - 1) * CHAR_BIT);
}

template <class T>
unsigned int hamming_distance(const T a, const T b)
{
    return count_set_bits(a ^ b);
}

template <int N>
struct BRIEF_Descriptor
{
    enum {BITS=N, WORDS = (N+31)/32 };

    uint32_t x[WORDS];
};

template <int N>
unsigned int hamming_distance(const BRIEF_Descriptor<N>& a, const BRIEF_Descriptor<N>& b)
{    
    unsigned int d = 0;
    for (int i=0; i<BRIEF_Descriptor<N>::WORDS; ++i)
        d += hamming_distance(a.x[i], b.x[i]);
    return d;
}


#endif
