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
#ifndef ECV_BRIEF_7_128_HPP
#define ECV_BRIEF_7_128_HPP

#include "stdint.h"

template <class T>
void BRIEF_7_128_extract(const T* p, int stride, uint32_t d[4])
{
    uint32_t b;
    { // word 0
        b = 0;
        if (p[-7*stride + -2] < p[-5*stride + 0]) b |= (1 << 0);
        if (p[7*stride + 6] < p[0*stride + 1]) b |= (1 << 1);
        if (p[0*stride + -6] < p[5*stride + 0]) b |= (1 << 2);
        if (p[1*stride + -5] < p[2*stride + 2]) b |= (1 << 3);
        if (p[-2*stride + -4] < p[0*stride + -1]) b |= (1 << 4);
        if (p[6*stride + 3] < p[1*stride + 0]) b |= (1 << 5);
        if (p[5*stride + -3] < p[3*stride + -3]) b |= (1 << 6);
        if (p[4*stride + 5] < p[-6*stride + -2]) b |= (1 << 7);
        if (p[-3*stride + -1] < p[3*stride + 6]) b |= (1 << 8);
        if (p[6*stride + 3] < p[5*stride + 0]) b |= (1 << 9);
        if (p[3*stride + 3] < p[-4*stride + 4]) b |= (1 << 10);
        if (p[-3*stride + 0] < p[7*stride + -4]) b |= (1 << 11);
        if (p[-7*stride + 2] < p[-4*stride + -1]) b |= (1 << 12);
        if (p[6*stride + -1] < p[2*stride + 3]) b |= (1 << 13);
        if (p[-1*stride + 1] < p[-1*stride + 5]) b |= (1 << 14);
        if (p[0*stride + -2] < p[5*stride + -5]) b |= (1 << 15);
        if (p[6*stride + 1] < p[6*stride + -3]) b |= (1 << 16);
        if (p[4*stride + -1] < p[-7*stride + 7]) b |= (1 << 17);
        if (p[1*stride + -3] < p[1*stride + 0]) b |= (1 << 18);
        if (p[0*stride + -5] < p[-5*stride + 0]) b |= (1 << 19);
        if (p[5*stride + -5] < p[6*stride + 5]) b |= (1 << 20);
        if (p[3*stride + 5] < p[-3*stride + 4]) b |= (1 << 21);
        if (p[-5*stride + 3] < p[7*stride + 4]) b |= (1 << 22);
        if (p[-5*stride + 1] < p[-6*stride + -6]) b |= (1 << 23);
        if (p[0*stride + 1] < p[-2*stride + -4]) b |= (1 << 24);
        if (p[-1*stride + -5] < p[1*stride + 7]) b |= (1 << 25);
        if (p[0*stride + 4] < p[-1*stride + 0]) b |= (1 << 26);
        if (p[6*stride + 0] < p[-4*stride + 2]) b |= (1 << 27);
        if (p[0*stride + -6] < p[-7*stride + -6]) b |= (1 << 28);
        if (p[5*stride + 0] < p[-6*stride + -3]) b |= (1 << 29);
        if (p[2*stride + 3] < p[0*stride + 7]) b |= (1 << 30);
        if (p[-6*stride + -2] < p[3*stride + -1]) b |= (1 << 31);
    }
    d[0] = b;
    { // word 1
        b = 0;
        if (p[0*stride + 2] < p[2*stride + 3]) b |= (1 << 0);
        if (p[2*stride + 2] < p[0*stride + -1]) b |= (1 << 1);
        if (p[6*stride + -1] < p[-3*stride + -2]) b |= (1 << 2);
        if (p[0*stride + 3] < p[-7*stride + -2]) b |= (1 << 3);
        if (p[5*stride + 5] < p[-4*stride + 3]) b |= (1 << 4);
        if (p[0*stride + -2] < p[0*stride + 3]) b |= (1 << 5);
        if (p[0*stride + -3] < p[1*stride + -1]) b |= (1 << 6);
        if (p[1*stride + 5] < p[5*stride + 1]) b |= (1 << 7);
        if (p[6*stride + 3] < p[-5*stride + 2]) b |= (1 << 8);
        if (p[0*stride + 0] < p[-3*stride + 2]) b |= (1 << 9);
        if (p[5*stride + 7] < p[-1*stride + -2]) b |= (1 << 10);
        if (p[2*stride + 0] < p[-1*stride + -5]) b |= (1 << 11);
        if (p[0*stride + 0] < p[-1*stride + -3]) b |= (1 << 12);
        if (p[-2*stride + 0] < p[-4*stride + 2]) b |= (1 << 13);
        if (p[6*stride + 4] < p[-6*stride + 0]) b |= (1 << 14);
        if (p[1*stride + 1] < p[0*stride + 0]) b |= (1 << 15);
        if (p[-2*stride + -5] < p[-5*stride + 7]) b |= (1 << 16);
        if (p[-4*stride + -6] < p[-4*stride + 2]) b |= (1 << 17);
        if (p[2*stride + -7] < p[7*stride + 4]) b |= (1 << 18);
        if (p[5*stride + 0] < p[1*stride + 2]) b |= (1 << 19);
        if (p[0*stride + -2] < p[-7*stride + 7]) b |= (1 << 20);
        if (p[-5*stride + 2] < p[-7*stride + 1]) b |= (1 << 21);
        if (p[3*stride + -6] < p[-5*stride + -5]) b |= (1 << 22);
        if (p[-3*stride + 1] < p[-1*stride + 6]) b |= (1 << 23);
        if (p[4*stride + 1] < p[0*stride + 1]) b |= (1 << 24);
        if (p[6*stride + 5] < p[-4*stride + 7]) b |= (1 << 25);
        if (p[-4*stride + -1] < p[0*stride + 0]) b |= (1 << 26);
        if (p[0*stride + 1] < p[7*stride + -7]) b |= (1 << 27);
        if (p[-1*stride + -1] < p[-6*stride + -5]) b |= (1 << 28);
        if (p[-5*stride + -6] < p[3*stride + -3]) b |= (1 << 29);
        if (p[-6*stride + -2] < p[7*stride + 7]) b |= (1 << 30);
        if (p[0*stride + -1] < p[-1*stride + -3]) b |= (1 << 31);
    }
    d[1] = b;
    { // word 2
        b = 0;
        if (p[6*stride + 3] < p[4*stride + -2]) b |= (1 << 0);
        if (p[5*stride + 0] < p[3*stride + -6]) b |= (1 << 1);
        if (p[0*stride + -4] < p[1*stride + 5]) b |= (1 << 2);
        if (p[0*stride + -7] < p[0*stride + 4]) b |= (1 << 3);
        if (p[-3*stride + 1] < p[4*stride + 3]) b |= (1 << 4);
        if (p[-5*stride + -2] < p[-7*stride + 0]) b |= (1 << 5);
        if (p[-2*stride + -4] < p[-2*stride + -6]) b |= (1 << 6);
        if (p[-4*stride + 4] < p[-4*stride + -5]) b |= (1 << 7);
        if (p[0*stride + 0] < p[5*stride + -1]) b |= (1 << 8);
        if (p[6*stride + 4] < p[-7*stride + -3]) b |= (1 << 9);
        if (p[7*stride + 1] < p[5*stride + -6]) b |= (1 << 10);
        if (p[2*stride + 3] < p[0*stride + -1]) b |= (1 << 11);
        if (p[5*stride + 0] < p[-2*stride + 7]) b |= (1 << 12);
        if (p[6*stride + 5] < p[-3*stride + 2]) b |= (1 << 13);
        if (p[2*stride + 3] < p[-7*stride + 0]) b |= (1 << 14);
        if (p[-2*stride + -6] < p[0*stride + -4]) b |= (1 << 15);
        if (p[0*stride + 6] < p[-7*stride + 5]) b |= (1 << 16);
        if (p[0*stride + -2] < p[3*stride + -3]) b |= (1 << 17);
        if (p[3*stride + 5] < p[-6*stride + -3]) b |= (1 << 18);
        if (p[6*stride + -4] < p[-3*stride + 0]) b |= (1 << 19);
        if (p[7*stride + 7] < p[-5*stride + -7]) b |= (1 << 20);
        if (p[6*stride + 6] < p[7*stride + -1]) b |= (1 << 21);
        if (p[5*stride + 7] < p[-4*stride + 2]) b |= (1 << 22);
        if (p[3*stride + -1] < p[-3*stride + -1]) b |= (1 << 23);
        if (p[2*stride + 0] < p[-5*stride + 0]) b |= (1 << 24);
        if (p[6*stride + 3] < p[-7*stride + 3]) b |= (1 << 25);
        if (p[6*stride + 5] < p[-7*stride + -6]) b |= (1 << 26);
        if (p[0*stride + -2] < p[-6*stride + 0]) b |= (1 << 27);
        if (p[-7*stride + -1] < p[-3*stride + -3]) b |= (1 << 28);
        if (p[1*stride + 6] < p[1*stride + -3]) b |= (1 << 29);
        if (p[7*stride + 4] < p[-4*stride + 2]) b |= (1 << 30);
        if (p[3*stride + 2] < p[0*stride + 3]) b |= (1 << 31);
    }
    d[2] = b;
    { // word 3
        b = 0;
        if (p[-3*stride + -3] < p[7*stride + 5]) b |= (1 << 0);
        if (p[2*stride + -3] < p[4*stride + 0]) b |= (1 << 1);
        if (p[-1*stride + -2] < p[-7*stride + 0]) b |= (1 << 2);
        if (p[1*stride + 1] < p[-1*stride + -5]) b |= (1 << 3);
        if (p[4*stride + 0] < p[-6*stride + 7]) b |= (1 << 4);
        if (p[7*stride + 5] < p[6*stride + -3]) b |= (1 << 5);
        if (p[7*stride + -1] < p[1*stride + 4]) b |= (1 << 6);
        if (p[1*stride + -6] < p[-3*stride + -3]) b |= (1 << 7);
        if (p[1*stride + 0] < p[6*stride + -3]) b |= (1 << 8);
        if (p[0*stride + -1] < p[6*stride + -2]) b |= (1 << 9);
        if (p[5*stride + 2] < p[0*stride + 5]) b |= (1 << 10);
        if (p[0*stride + 0] < p[-7*stride + 2]) b |= (1 << 11);
        if (p[-7*stride + -3] < p[5*stride + 0]) b |= (1 << 12);
        if (p[5*stride + 0] < p[0*stride + 1]) b |= (1 << 13);
        if (p[-2*stride + -6] < p[-2*stride + -3]) b |= (1 << 14);
        if (p[7*stride + 3] < p[0*stride + 2]) b |= (1 << 15);
        if (p[3*stride + 4] < p[5*stride + 7]) b |= (1 << 16);
        if (p[-3*stride + 7] < p[6*stride + 0]) b |= (1 << 17);
        if (p[-5*stride + -2] < p[0*stride + -1]) b |= (1 << 18);
        if (p[0*stride + 2] < p[4*stride + 5]) b |= (1 << 19);
        if (p[0*stride + 0] < p[4*stride + -4]) b |= (1 << 20);
        if (p[6*stride + -2] < p[0*stride + 3]) b |= (1 << 21);
        if (p[-2*stride + 3] < p[1*stride + 3]) b |= (1 << 22);
        if (p[1*stride + -5] < p[-4*stride + -3]) b |= (1 << 23);
        if (p[7*stride + 2] < p[-1*stride + 0]) b |= (1 << 24);
        if (p[1*stride + -3] < p[-4*stride + 7]) b |= (1 << 25);
        if (p[-4*stride + 1] < p[2*stride + 0]) b |= (1 << 26);
        if (p[-6*stride + -2] < p[2*stride + 4]) b |= (1 << 27);
        if (p[5*stride + 6] < p[2*stride + 5]) b |= (1 << 28);
        if (p[2*stride + -3] < p[5*stride + -1]) b |= (1 << 29);
        if (p[2*stride + 5] < p[-3*stride + 5]) b |= (1 << 30);
        if (p[3*stride + 1] < p[-2*stride + 1]) b |= (1 << 31);
    }
    d[3] = b;
}

#endif
