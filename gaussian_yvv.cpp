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
#include <ecv/gaussian_yvv.hpp>
#include <latl/ldlt.hpp>
#include <vector>

using namespace ecv;
using namespace std;
using namespace latl;

static inline
double gaussian_ref(double w, double sigma)
{
    double ws = w * sigma;
    return exp(-0.5 * ws*ws);
}

static
double eval_H3(const double d[3], double w, double J[3])
{    
    double cw = cos(w);
    double isq = sq(d[1]);
    double cwrsq = sq(cw - d[0]);

    double num01r = sq(d[0]-1) + isq;
    double num01 = sq(num01r);
    double Jn0 = num01r*(d[0]-1);
    double Jn1 = num01r*d[1];
    
    double den01r = isq - cwrsq + cw*cw - 1;
    double den01 = 4*isq*cwrsq + sq(den01r);
    double Jd0 = (cw-d[0])*(-2*isq + den01r);
    double Jd1 = d[1]*(2*cwrsq + den01r);

    double num2 = sq(d[2]-1);
    double den2 = 2*d[2]*cw - 1 - sq(d[2]);
    double Jn2 = d[2]-1;
    double Jd2 = cw - d[2];

    double inv_den01 = 1/den01;
    double inv_den2 = 1/den2;
    double A = num01 * inv_den01;    
    double B = num2 * inv_den2;
    
    J[0] = -B * inv_den01*4*(Jn0 - A*Jd0);
    J[1] = -B * inv_den01*4*(Jn1 - A*Jd1);
    J[2] = -A *  inv_den2*2*(Jn2 - B*Jd2);
    
    return -A*B;
}

static
double refine_H3(double d[], double sigma, double lambda)
{
    const int segs = 1000;
    double dw = 2*M_PI / segs;

    Matrix<3,3> A(0.0);
    Vector<3> b(0.0);
    Vector<3> J;
    
    double residual = 0;
    for (int i=0; i<segs; ++i) {
        double w = dw * i - M_PI;
        double z = gaussian_ref(w, sigma);
        double h = eval_H3(d, w, &J[0]);
        double v = z - h;
        outer_product_upper(J, ops::Add(), A);
        b += J * v;
        residual += v*v;
    }
    
    for (int i=0; i<3; ++i)
        A(i,i) += lambda;
    
    LDLT<3,double> ldlt(A);
    assert(ldlt.is_full_rank());

    Vector<3> update = ldlt.inverse_times(b);
    for (int i=0; i<3; ++i)
        d[i] += update[i];
    
    return residual * dw / (2*M_PI);
}

static
void scale_d3(double d[], double q)
{
    double inv_q = 1/q;
    double theta = atan2(d[1], d[0]) * inv_q;
    double ct = cos(theta);
    double st = sin(theta);
    double r0 = pow(d[0]*d[0] + d[1]*d[1], 0.5*inv_q);
    double r1 = pow(fabs(d[2]), inv_q);
    d[0] = r0*ct;
    d[1] = r0*st;
    d[2] = r1;
}

static
void compute_b(double d[3], double b[3])
{
    double d01_sq = d[0]*d[0] + d[1]*d[1];
    double B = 1/(d01_sq * d[2]);
    b[0] = -B * (d01_sq + 2*d[0]*d[2]);
    b[1] =  B * (2*d[0] + d[2]);
    b[2] = -B;
}

static
void compute_b_for_sigma(double sigma, double b[3])
{
    // For sigma = 2.0
    double d[3] = {1.329837043201809399,
                   1.025761683320574491,
                   1.745087663306153836};
    
    scale_d3(d, sigma * 0.5);
    
    for (int pass=0; pass < 10; ++pass) {
        refine_H3(d, sigma, 1e-2);
    }
    
    compute_b(d, b);    
}    

void ecv::YvV_Params::init(double sigma_)
{
    sigma = sigma_;
        
    double b[3];
    compute_b_for_sigma(sigma, b);

    b0 = -b[0];
    b1 = -b[1];
    b2 = -b[2];

    alpha = 1 - (b0 + b1 + b2);
    float f = 1 / ((1+b0-b1+b2) * (1 + b1 + (b0-b2)*b2));

    m[0] = f*(1 - b1 - b2*(b0 + b2));
    m[1] = f*((b0 + b2)*(b1 + b0*b2));
    m[2] = f*(b2*(b0 + b1*b2));

    m[3] = f*(b0 + b1*b2);
    m[4] = f*((1 - b1)*(b1 + b0*b2));
    m[5] = -f*b2*(b2*(b0 + b2) + b1 - 1);

    m[6] = f*(b0*b2 + b1 + b0*b0 - b1*b1);
    m[7] = f*(b0*b1 + b2*(b1*b1 - b2*(b0 + b2) - b1 + 1));
    m[8] = m[2];
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

void ecv::yvv_blur_horizontal(Image<float>& im, const YvV_Params& yvv)
{
    for (int i=0; i+1<im.height(); i+=2) {
        float* p = im[i];
        float* n = im[i+1];
        float uplus = p[im.width()-1];
        float uplus1 = n[im.width()-1];
        
        {
            float v = p[0];
            float w = n[0];
            p[1] = yvv.alpha*p[1] + (yvv.b0*p[0] + (yvv.b1 + yvv.b2)*v);
            n[1] = yvv.alpha*n[1] + (yvv.b0*n[0] + (yvv.b1 + yvv.b2)*w);
            p[2] = yvv.alpha*p[2] + (yvv.b0*p[1] + yvv.b1*p[0] + yvv.b2*v);
            n[2] = yvv.alpha*n[2] + (yvv.b0*n[1] + yvv.b1*n[0] + yvv.b2*w);
            p += 3;
            n += 3;
        }

        int j = 3;
        {
            float u0=p[-1], u1=p[-2], u2=p[-3];
            float t0=n[-1], t1=n[-2], t2=n[-3];
            for (; j+2<im.width(); j+=3, p+=3, n+=3) {
                p[0] = u2 = yvv.alpha*p[0] + (yvv.b0*u0 + yvv.b1*u1 + yvv.b2*u2);
                n[0] = t2 = yvv.alpha*n[0] + (yvv.b0*t0 + yvv.b1*t1 + yvv.b2*t2);
                p[1] = u1 = yvv.alpha*p[1] + (yvv.b0*u2 + yvv.b1*u0 + yvv.b2*u1);
                n[1] = t1 = yvv.alpha*n[1] + (yvv.b0*t2 + yvv.b1*t0 + yvv.b2*t1);
                p[2] = u0 = yvv.alpha*p[2] + (yvv.b0*u1 + yvv.b1*u2 + yvv.b2*u0);
                n[2] = t0 = yvv.alpha*n[2] + (yvv.b0*t1 + yvv.b1*t2 + yvv.b2*t0);
            }
        }
        for (; j<im.width(); ++j, ++p, ++n) {
            p[0] = yvv.alpha*p[0] + (yvv.b0*p[-1] + yvv.b1*p[-2] + yvv.b2*p[-3]);
            n[0] = yvv.alpha*n[0] + (yvv.b0*n[-1] + yvv.b1*n[-2] + yvv.b2*n[-3]);
        }

        --p;
        --n;
        
        {
            float u[3] = {p[0], p[-1], p[-2]};
            float t[3] = {n[0], n[-1], n[-2]};
            float v[3], w[3];
            yvv_flip(yvv, uplus, u, v); 
            yvv_flip(yvv, uplus1, t, w);
           
            p[ 0] = v[0];
            n[ 0] = w[0];
            p[-1] = yvv.alpha*p[-1] + (yvv.b0*v[0] + yvv.b1*v[1] + yvv.b2*v[2]);
            n[-1] = yvv.alpha*n[-1] + (yvv.b0*w[0] + yvv.b1*w[1] + yvv.b2*w[2]);
            p[-2] = yvv.alpha*p[-2] + (yvv.b0*p[-1] + yvv.b1*v[0] + yvv.b2*v[1]);
            n[-2] = yvv.alpha*n[-2] + (yvv.b0*n[-1] + yvv.b1*w[0] + yvv.b2*w[1]);
            p -= 3;
            n -= 3;
        }

        j = 3;        
        {
            float v0=p[1], v1=p[2], v2=p[3];
            float w0=n[1], w1=n[2], w2=n[3];
            for (; j+2<im.width(); j+=3, p-=3, n-=3) {
                p[ 0] = v2 = yvv.alpha*p[ 0] + (yvv.b0*v0 + yvv.b1*v1 + yvv.b2*v2);
                n[ 0] = w2 = yvv.alpha*n[ 0] + (yvv.b0*w0 + yvv.b1*w1 + yvv.b2*w2);
                p[-1] = v1 = yvv.alpha*p[-1] + (yvv.b0*v2 + yvv.b1*v0 + yvv.b2*v1);
                n[-1] = w1 = yvv.alpha*n[-1] + (yvv.b0*w2 + yvv.b1*w0 + yvv.b2*w1);
                p[-2] = v0 = yvv.alpha*p[-2] + (yvv.b0*v1 + yvv.b1*v2 + yvv.b2*v0);
                n[-2] = w0 = yvv.alpha*n[-2] + (yvv.b0*w1 + yvv.b1*w2 + yvv.b2*w0);
            }
        }
        for (; j<im.width(); ++j, --p, --n) {
            p[0] = yvv.alpha*p[0] + (yvv.b0*p[1] + yvv.b1*p[2] + yvv.b2*p[3]);
            n[0] = yvv.alpha*n[0] + (yvv.b0*n[1] + yvv.b1*n[2] + yvv.b2*n[3]);
        }
    }
}

void ecv::yvv_blur_vertical(Image<float>& im, const YvV_Params& yvv)
{
    {
        const float* p0 = im[0];
        float* p1 = im[1];
        float* p2 = im[2];
        for (int j=0; j<im.width(); ++j) {
            p1[j] = yvv.alpha*p1[j] + (yvv.b0 + yvv.b1 + yvv.b2)*p0[j];
            p2[j] = yvv.alpha*p2[j] + yvv.b0*p1[j] + (yvv.b1 + yvv.b2)*p0[j];
        }
    }

    vector<float> uplus(im[im.height()-1], im[im.height()-1] + im.width());
    
    const int s = im.stride();
    for (int i=3; i<im.height(); ++i) {
        float* p = im[i];
        const float* p2 = p - 2*s;

        for (int j=0; j<im.width(); ++j) {
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
        float* p = im[i];
        const float* p2 = p + 2*s;
        
        for (int j=0; j<im.width(); ++j) {
            p[j] = yvv.alpha*p[j] + yvv.b0*p[j+s] + yvv.b1*p2[j] + yvv.b2*p2[j+s];
        }
    }
}

#if 0

#include <xmmintrin.h>

void blur_vertical_sse(Image<float>& im, const YvV_Params& yvv)
{

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

#define TRANSPOSE(a,b,c,d) \
    {                                           \
    __m128 t = _mm_unpackhi_ps(c, d);           \
    c = _mm_unpacklo_ps(c, d);                  \
    d = _mm_unpackhi_ps(a, b);                  \
    a = _mm_unpacklo_ps(a, b);                  \
    b = (__m128)_mm_unpackhi_pd((__m128d)a, (__m128d)c);          \
    a = (__m128)_mm_unpacklo_pd((__m128d)a, (__m128d)c);          \
    c = (__m128)_mm_unpacklo_pd((__m128d)d, (__m128d)t);                  \
    d = (__m128)_mm_unpackhi_pd((__m128d)d, (__m128d)t);                  \
    }

#endif

void ecv::yvv_blur(Image<float>& im, const YvV_Params& yvv)
{
    yvv_blur_horizontal(im, yvv);
    yvv_blur_vertical(im, yvv);
}

void ecv::yvv_blur(Image<float>& im, double sigma)
{
    YvV_Params yvv;
    yvv.init(sigma);
    yvv_blur(im, yvv);
}

#if 0

#include <ecv/io_pgm.hpp>
#include <ecv/convert.hpp>
#include <fstream>
#include <ecv/timer.hpp>

int main(int argc, char * argv[])
{
    double sigma = 10.0;
    YvV_Params yvv(sigma);
    
    Image<uint8_t> im;
    {        
        ifstream in(argv[1]);
        pgm_read(in, im);
    }

    Image<float> imf = convert<float>(im);

    Timer timer;
    yvv_blur_vertical_sse(imf, yvv);
    //blur_horizontal(imf, yvv);
    cerr << timer.get_time() * 1e3 << " ms" << endl;

    im = convert<uint8_t>(imf);

    {
        ofstream out("blurred.pgm");
        pgm_write(im, out);
    }    
    
    return 0;
}
#endif
