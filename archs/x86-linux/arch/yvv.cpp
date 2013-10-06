#include <arch/yvv.hpp>
#include <xmmintrin.h>
#include <vector>

using namespace std;
using namespace ecv;


static bool is_aligned_16(const void* p)
{
    return (((const char*)p - (const char*)0)&0xF) == 0;
}

static
void yvv_flip(const YvV_Params& p, float uplus, float u[3], float v[3])
{
    float u0 = u[0] - uplus;
    float u1 = u[1] - uplus;
    float u2 = u[2] - uplus;

    v[0] = p.m[0]*u0 + p.m[1]*u1 + p.m[2]*u2 + uplus;
    v[1] = p.m[3]*u0 + p.m[4]*u1 + p.m[5]*u2 + uplus;
    v[2] = p.m[6]*u0 + p.m[7]*u1 + p.m[8]*u2 + uplus;
}

void ecv::arch::yvv_blur_vertical(Image<float>& im, const YvV_Params& yvv)
{
    if (!is_aligned_16(im[0]) || !is_aligned_16(im[1])) {
        ecv::yvv_blur_vertical_ref(im, yvv);
        return;
    }
        
    const __m128 aaaa = _mm_set1_ps(yvv.alpha);
    const __m128 bbb0 = _mm_set1_ps(yvv.b0);
    const __m128 bbb1 = _mm_set1_ps(yvv.b1);
    const __m128 bbb2 = _mm_set1_ps(yvv.b2);

    const int s = im.stride();
    const int s4 = s/4;
    const int w4 = im.width()/4;
    {
        __m128* pppp0 = (__m128*)im[0];
        __m128* pppp1 = pppp0 + s4;
        __m128* pppp2 = pppp1 + s4;
        for (int j=0; j<w4; ++j) {
            pppp1[j] = aaaa*pppp1[j] + (bbb0 + bbb1 + bbb2)*pppp0[j];
            pppp2[j] = aaaa*pppp2[j] + bbb0*pppp1[j] + (bbb1 + bbb2)*pppp0[j];
        }
        const float *p0 = im[0];
        float *p1 = im[1];
        float *p2 = im[2];
        for (int j=w4*4; j<im.width(); ++j) {
            p1[j] = yvv.alpha*p1[j] + (yvv.b0 + yvv.b1 + yvv.b2)*p0[j];
            p2[j] = yvv.alpha*p2[j] + yvv.b0*p1[j] + (yvv.b1 + yvv.b2)*p0[j];
        }
    }

    vector<float> uplus(im[im.height()-1], im[im.height()-1] + im.width());
    
    for (int i=3; i<im.height(); ++i) {
        __m128* pppp = (__m128*)im[i];
        const __m128* pppp2 = (__m128*)im[i-2];

        for (int j=0; j<w4; ++j) {
            pppp[j] = aaaa*pppp[j] + bbb0*pppp[j-s4] + bbb1*pppp2[j] + bbb2*pppp2[j-s4];
        }
        float *p = im[i];
        const float *p2 = p - 2*s;
        for (int j=w4*4; j<im.width(); ++j) {
            p[j] = yvv.alpha*p[j] + yvv.b0*p[j-s] + yvv.b1*p2[j] + yvv.b2*p2[j-s];
        }
    }


    {
        float *p = im[im.height()-1];
        float *p2 = p - 2*s;
        for (int j=0; j<im.width(); ++j) {
            float u[3] = {p[j], p[j-s], p2[j]};
            float v[3];
            yvv_flip(yvv, uplus[j], u, v);
            p[  j] = v[0];
            p[j-s] = yvv.alpha*p[j-s] + yvv.b0*v[0] + yvv.b1*v[1] + yvv.b2*v[2];
            p2[ j] = yvv.alpha*p2[ j] + yvv.b0*p[j-s] + yvv.b1*v[0] + yvv.b2*v[1];
        }
    }

    for (int i=im.height()-4; i>=0; --i) {
        __m128* pppp = (__m128*)im[i];
        const __m128* pppp2 = (__m128*)im[i+2];
        
        for (int j=0; j<w4; ++j) {
            pppp[j] = aaaa*pppp[j] + bbb0*pppp[j+s4] + bbb1*pppp2[j] + bbb2*pppp2[j+s4];
        }
        float *p = im[i];
        const float *p2 = p + s*2;
        for (int j=w4*4; j<im.width(); ++j) {
            p[j] = yvv.alpha*p[j] + yvv.b0*p[j+s] + yvv.b1*p2[j] + yvv.b2*p2[j+s];
        }
    }
}

#define TRANSPOSE(a,b,c,d)                                              \
    {                                                                   \
        __m128 t = _mm_unpackhi_ps(c, d);                               \
        c = _mm_unpacklo_ps(c, d);                                      \
        d = _mm_unpackhi_ps(a, b);                                      \
        a = _mm_unpacklo_ps(a, b);                                      \
        b = (__m128)_mm_unpackhi_pd((__m128d)a, (__m128d)c);            \
        a = (__m128)_mm_unpacklo_pd((__m128d)a, (__m128d)c);            \
        c = (__m128)_mm_unpacklo_pd((__m128d)d, (__m128d)t);            \
        d = (__m128)_mm_unpackhi_pd((__m128d)d, (__m128d)t);            \
    }

static void interleave_four_rows(const float *p, int width, int stride,
                                 __m128 *buf)
{
    const float *pa = p;
    const float *pc = p + stride*2;

    int i;
    for (i=0; i+3<width; i+=4) {
        __m128 a = _mm_load_ps(pa+i);
        __m128 b = _mm_load_ps(pa+i+stride);
        __m128 c = _mm_load_ps(pc+i);
        __m128 d = _mm_load_ps(pc+i+stride);
        TRANSPOSE(a,b,c,d);
        buf[0] = a;
        buf[1] = b;
        buf[2] = c;
        buf[3] = d;
        buf += 4;
    }

    for (; i<width; ++i) {
        *buf++ = _mm_set_ps(pa[i], pa[i+stride], pc[i], pc[i+stride]);
    }     
}

static void deinterleave_four_rows(const __m128 *buf, int width, int stride,
                                   float *p)
{
    float *pa = p;
    float *pc = p + stride*2;

    int i;
    for (i=0; i+3<width; i+=4) {
        __m128 a = buf[0];
        __m128 b = buf[1];
        __m128 c = buf[2];
        __m128 d = buf[3];
        buf += 4;
        
        TRANSPOSE(a,b,c,d);

        _mm_store_ps(pa+i, a);
        _mm_store_ps(pa+i+stride, b);
        _mm_store_ps(pc+i, c);
        _mm_store_ps(pc+i+stride, d);
    }

    float abcd[4] __attribute__((aligned(16)));
    for (; i<width; ++i) {
        _mm_store_ps(abcd, *buf++);
        pa[i] = abcd[0];
        pa[i+stride] = abcd[1];
        pc[i] = abcd[2];
        pc[i+stride] = abcd[3];
    }     
}

static
void yvv_flip4(const YvV_Params& p, __m128 uplus, __m128 u[3], __m128 v[3])
{
    __m128 u0 = u[0] - uplus;
    __m128 u1 = u[1] - uplus;
    __m128 u2 = u[2] - uplus;

    for (int i=0; i<3; ++i) {
        __m128 a = _mm_set1_ps(p.m[i*3]);
        __m128 b = _mm_set1_ps(p.m[i*3+1]);
        __m128 c = _mm_set1_ps(p.m[i*3+2]);
        v[i] = a*u0 + b*u1 + c*u2 + uplus;
    }
}

void ecv::arch::yvv_blur_horizontal(Image<float>& im, const YvV_Params& yvv)
{
    if (!is_aligned_16(im[0]) || !is_aligned_16(im[1])) {
        ecv::yvv_blur_horizontal_ref(im, yvv);
        return;
    }
    
    const __m128 aaaa = _mm_set1_ps(yvv.alpha);
    const __m128 bbb0 = _mm_set1_ps(yvv.b0);
    const __m128 bbb1 = _mm_set1_ps(yvv.b1);
    const __m128 bbb2 = _mm_set1_ps(yvv.b2);

    Image<float> interleaved_buf(im.width()*4,1);
    __m128 *row_buf = (__m128*)interleaved_buf[0];

    for (int i=0; i+3<im.height(); i+=4) {
        float* row = im[i];
        interleave_four_rows(row, im.width(), im.stride(), row_buf);

        __m128 *p = row_buf;
        
        __m128 uplus = p[im.width()-1];
        {
            __m128 v = p[0];
            p[1] = aaaa*p[1] + (bbb0*p[0] + (bbb1 + bbb2)*v);
            p[2] = aaaa*p[2] + (bbb0*p[1] + bbb1*p[0] + bbb2*v);
            p += 3;
        }

        int j = 3;
        {
            __m128 u0=p[-1], u1=p[-2], u2=p[-3];
            for (; j+2<im.width(); j+=3, p+=3) {
                p[0] = u2 = aaaa*p[0] + (bbb0*u0 + bbb1*u1 + bbb2*u2);
                p[1] = u1 = aaaa*p[1] + (bbb0*u2 + bbb1*u0 + bbb2*u1);
                p[2] = u0 = aaaa*p[2] + (bbb0*u1 + bbb1*u2 + bbb2*u0);
            }
        }
        for (; j<im.width(); ++j, ++p) {
            p[0] = aaaa*p[0] + (bbb0*p[-1] + bbb1*p[-2] + bbb2*p[-3]);
        }

        --p;
        
        {
            __m128 u[3] = {p[0], p[-1], p[-2]};
            __m128 v[3];
            yvv_flip4(yvv, uplus, u, v); 
           
            p[ 0] = v[0];
            p[-1] = aaaa*p[-1] + (bbb0*v[0] + bbb1*v[1] + bbb2*v[2]);
            p[-2] = aaaa*p[-2] + (bbb0*p[-1] + bbb1*v[0] + bbb2*v[1]);
            p -= 3;
        }

        j = 3;        
        {
            __m128 v0=p[1], v1=p[2], v2=p[3];
            for (; j+2<im.width(); j+=3, p-=3) {
                p[ 0] = v2 = aaaa*p[ 0] + (bbb0*v0 + bbb1*v1 + bbb2*v2);
                p[-1] = v1 = aaaa*p[-1] + (bbb0*v2 + bbb1*v0 + bbb2*v1);
                p[-2] = v0 = aaaa*p[-2] + (bbb0*v1 + bbb1*v2 + bbb2*v0);
            }
        }
        for (; j<im.width(); ++j, --p) {
            p[0] = aaaa*p[0] + (bbb0*p[1] + bbb1*p[2] + bbb2*p[3]);
        }
        
        deinterleave_four_rows(row_buf, im.width(), im.stride(), row);        
    }

    int rows_left = im.height() % 4;
    if (rows_left) {
        Image<float> bottom = im.sub_image(0, (im.height()/4)*4, im.width(), rows_left);
        yvv_blur_horizontal_ref(bottom, yvv);
    }
}
