#include <ecv/subsample.hpp>

using namespace std;
using namespace ecv;

template <int Low, int High>
inline int clamp(int x)
{
    return x < 0 ? Low : (x > High ? High : x);
}

void ecv::subsample_two_thirds_bicubic(const Image<uint8_t>& in, Image<uint8_t> &out)
{
    Image<int16_t> narrow((in.width()*2 + 2)/3, in.height());
    out.resize(narrow.width(), (in.height()*2 + 2)/3);    

    for (int i=0; i<in.height(); ++i) {
        const uint8_t *p = in[i];
        int16_t *o = narrow[i];

        int k = in.width();
        for (; k>3; k-=3) {
            o[0] = 16*p[0];
            o[1] = 9*(p[1] + p[2]) - (p[0] + p[3]);
            o += 2;
            p += 3;
        }

        switch (k) {
        case 3:
            o[0] = 16*p[0];
            o[1] = 9*(p[1] + p[2]) - (p[0] + p[2]);
            break;
        case 2:
            o[0] = 16*p[0];
            o[1] = 17*p[1] - p[0];
            break;
        case 1:
            o[0] = 16*p[0];
            break;
        case 0: break;
        }
    }

    const int s = narrow.stride();
    const int w = narrow.width();
    const int16_t *p = narrow[0];
    for (int i=0, k=0; i<out.height(); ++i, ++k)
    {
        uint8_t *o = out[i];
        
        const int16_t *r0 = k>0 ? p-s : p;
        const int16_t *r2 = k+1 < narrow.height() ? p+s : p;
        const int16_t *r3 = k+2 < narrow.height() ? r2+s : r2;

        switch (i%2) {
        case 0:
            for (int j=0; j<w; ++j) {
                int y = (p[j] + 8) >> 4;
                o[j] = clamp<0,255>(y);
            }
            p = r2;
            break;
        case 1:
            for (int j=0; j<w; ++j) {
                int y = (9*(p[j] + r2[j]) - (r0[j] + r3[j]) + (1<<7)) >> 8;
                o[j] = clamp<0,255>(y);
            }
            p = r3;
            ++k;
            break;
        }
    }
}

void ecv::subsample_four_fifths_bicubic(const Image<uint8_t>& in, Image<uint8_t> &out)
{
    Image<int16_t> narrow((in.width()*4 + 3)/5, in.height());
    out.resize(narrow.width(), (in.height()*4 + 3)/5);    

    for (int i=0; i<in.height(); ++i) {
        const uint8_t *p = in[i];
        int16_t *o = narrow[i];

        int k = in.width();
        for (; k>5; k-=5) {
            o[0] = 64*p[0];
            o[1] = (-9*p[0] + 111*p[1] + 29*p[2] - 3*p[3]) >> 1;
            o[2] = (72*(p[2] + p[3]) - 8*(p[1] + p[4])) >> 1;
            o[3] = (-3*p[2] + 29*p[3] + 111*p[4] - 9*p[5]) >> 1;
            o += 4;
            p += 5;
        }

        if (k == 0)
            continue;
        
        o[0] = 64*p[0];

        switch (k) {
        case 5:
            o[1] = (-9*p[0] + 111*p[1] + 29*p[2] - 3*p[3]) >> 1;
            o[2] = (72*(p[2] + p[3]) - 8*(p[1] + p[4])) >> 1;
            o[3] = (-3*p[2] + 29*p[3] + 102*p[4]) >> 1;
            break;
        case 4:
            o[1] = (-9*p[0] + 111*p[1] + 29*p[2] - 3*p[3]) >> 1;
            o[2] = (-8*p[1] + 72*p[2] + 64*p[3]) >> 1;
            break;
        case 3:
            o[1] = (-9*p[0] + 111*p[1] + 26*p[2]) >> 1;
            o[2] = (-8*p[1] + 136*p[2]) >> 1;
            break;
        case 2:
            o[1] = (-9*p[0] + 137*p[1]) >> 1;
            break;
        case 1: break;
        }
    }

    const int s = narrow.stride();
    const int w = narrow.width();
    const int16_t *p = narrow[0];
    for (int i=0, k=0; i<out.height(); ++i, ++k)
    {
        uint8_t *o = out[i];
        
        const int16_t *r0 = k>0 ? p-s : p;
        const int16_t *r2 = k+1 < narrow.height() ? p+s : p;
        const int16_t *r3 = k+2 < narrow.height() ? r2+s : r2;

        switch (i%4) {
        case 0:
            for (int j=0; j<w; ++j) {
                int y = (p[j] + 32) >> 6;
                o[j] = clamp<0,255>(y);
            }
            p = r2;
            break;
        case 1:
            for (int j=0; j<w; ++j) {
                int y = (-9*r0[j] + 111*p[j] + 29*r2[j] - 3*r3[j] + (1<<12)) >> 13;
                o[j] = clamp<0,255>(y);
            }
            p = r2;
            break;
        case 2:
            for (int j=0; j<w; ++j) {
                int y = (72*(p[j] + r2[j]) - 8*(r0[j] + r3[j]) + (1<<12)) >> 13;
                o[j] = clamp<0,255>(y);
            }
            p = r2;
            break;
        case 3:
            for (int j=0; j<w; ++j) {
                int y = (-9*r3[j] + 111*r2[j] + 29*p[j] - 3*r0[j] + (1<<12)) >> 13;
                o[j] = clamp<0,255>(y);
            }
            p = r3;
            ++k;
            break;
        }
    }
}
