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
    enum {WORDS = (N+31)/32 };

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
