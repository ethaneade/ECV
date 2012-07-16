#include <ecv/convolution.hpp>
#include <vector>
#include <cmath>

using namespace std;
using namespace ecv;

template <int Bits, class T>
inline T rounded_shift(T x)
{
    return (x + (1<<(Bits-1))) >> Bits;
}

void ecv::convolve_121(const Image<uint8_t> &in, Image<uint8_t>& out)
{
    const int w = in.width();
    std::vector<uint16_t> buf(3*w);
    uint16_t *r0 = buf.data();
    uint16_t *r1 = r0, *r2 = r0;

    out.resize(in.size());
    for (int i=0; i<in.height()+1; ++i)
    {
        if (i < in.height()) {
            const uint8_t *p = in[i];
            uint16_t *o = r2;
            
            o[0] = 3*p[0] + p[1];
            for (int j=1; j<w-1; ++j) {
                o[j] = 2*p[j] + (p[j-1] + p[j+1]);            
            }
            o[w-1] = 3*p[w-1] + p[w-2];
        }
        
        if (i > 0) {
            uint8_t *o = out[i-1];
            for (int j=0; j<w; ++j) {
                o[j] = rounded_shift<4>(2*r1[j] +
                                        (r0[j] + r2[j]));
            }
        }

        uint16_t *old_r0 = r0;
        r0 = r1;
        r1 = r2;
        if (i+1 < in.height())
            r2 = i<2 ? r2 + w : old_r0;
    }    
}


void ecv::convolve_symmetric_3tap_12bit(const Image<uint8_t> &in,
                                        int kernel[2],
                                        Image<uint8_t> &out)
{
    const int w = in.width();
    std::vector<uint32_t> buf(3*w);
    uint32_t *r0 = buf.data();
    uint32_t *r1 = r0, *r2 = r0;

    const int A = kernel[0];
    const int B = kernel[1];
    const int Bits = 12;
    
    out.resize(in.size());
    for (int i=0; i<in.height()+1; ++i)
    {
        if (i < in.height()) {
            const uint8_t *p = in[i];
            uint32_t *o = r2;
            
            o[0] = (A+B)*p[0] + B*p[1];
            for (int j=1; j<w-1; ++j) {
                o[j] = A*p[j] + B*(p[j-1] + p[j+1]);            
            }
            o[w-1] = (A+B)*p[w-1] + B*p[w-2];
        }
        
        if (i > 0) {
            uint8_t *o = out[i-1];
            for (int j=0; j<w; ++j) {
                o[j] = rounded_shift<Bits*2>(A*r1[j] +
                                             B*(r0[j] + r2[j]));
            }
        }

        uint32_t *old_r0 = r0;
        r0 = r1;
        r1 = r2;
        if (i+1 < in.height())
            r2 = i<2 ? r2 + w : old_r0;
    }    
}

template <int Shift, typename Wider, typename In, typename Out>
static void convolve_symmetric_5tap(const Image<In> &in,
                                    int kernel[3],
                                    Image<Out> &out)
{
    const int w = in.width();
    std::vector<Wider> buf(5*w);
    Wider *r0 = buf.data();
    Wider *r1 = r0, *r2 = r0, *r3 = r0, *r4 = r0;

    const int A = kernel[0];
    const int B = kernel[1];
    const int C = kernel[2];
    
    out.resize(in.size());
    for (int i=0; i<in.height()+2; ++i)
    {
        if (i < in.height()) {
            const In *p = in[i];
            Wider *o = r4;
            
            o[0] = (A+B+C)*p[0] + B*p[1] + C*p[2];
            o[1] = A*p[1] + B*(p[0] + p[2]) + C*(p[0] + p[3]);
            int j;
            for (j=2; j<w-2; ++j) {
                o[j] = A*p[j] + B*(p[j-1] + p[j+1]) + C*(p[j-2] + p[j+2]);
            }
            o[j] = A*p[j] + B*(p[j-1] + p[j+1]) + C*(p[j-2] + p[j+1]);
            ++j;
            o[j] = A*p[j] + B*(p[j-1] + p[j]) + C*(p[j-2] + p[j]);
        }
        
        if (i > 1) {
            Out *o = out[i-2];
            for (int j=0; j<w; ++j) {
                o[j] = rounded_shift<Shift>(A*r2[j] +
                                            B*(r1[j] + r3[j]) +
                                            C*(r0[j] + r4[j]));
            }
        }

        Wider *old_r0 = r0;
        r0 = r1;
        r1 = r2;
        r2 = r3;
        r3 = r4;
        if (i<4)
            r4 += w;
        else if (i+1 < in.height())
            r4 = old_r0;
    }    
}

void ecv::convolve_symmetric_5tap_12bit(const Image<uint8_t> &in,
                                        int kernel[3],
                                        Image<uint8_t> &out)
{
    convolve_symmetric_5tap<24,uint32_t>(in, kernel, out);
}

void ecv::convolve_symmetric_5tap_12bit_8to12(const Image<uint8_t> &in,
                                              int kernel[3],
                                              Image<uint16_t> &out)
{
    convolve_symmetric_5tap<20,uint32_t>(in, kernel, out);
}

void ecv::convolve_symmetric_7tap_12bit(const Image<uint8_t> &in,
                                        int kernel[4],
                                        Image<uint8_t> &out)
{
    const int w = in.width();
    std::vector<uint32_t> buf(7*w);
    uint32_t *r[7];
    for (int i=0; i<7; ++i)
        r[i] = buf.data();

    const int A = kernel[0];
    const int B = kernel[1];
    const int C = kernel[2];
    const int D = kernel[3];
    const int Bits = 12;
    
    out.resize(in.size());
    for (int i=0; i<in.height()+3; ++i)
    {
        if (i < in.height()) {
            const uint8_t *p = in[i];
            uint32_t *o = r[6];
            
            o[0] = (A+B+C+D)*p[0] + B*p[1] + C*p[2] + D*p[3];
            o[1] = A*p[1] + B*(p[0] + p[2]) + C*(p[0] + p[3]) + D*(p[0] + p[4]);
            o[2] = A*p[2] + B*(p[1] + p[3]) + C*(p[0] + p[4]) + D*(p[0] + p[5]);
            int j;
            for (j=3; j<w-3; ++j) {
                o[j] = A*p[j] + B*(p[j-1] + p[j+1]) + C*(p[j-2] + p[j+2]) + D*(p[j-3] + p[j+3]);
            }
            o[j] = A*p[j] + B*(p[j-1] + p[j+1]) + C*(p[j-2] + p[j+2]) + D*(p[j-3] + p[j+2]);
            ++j;
            o[j] = A*p[j] + B*(p[j-1] + p[j+1]) + C*(p[j-2] + p[j+1]) + D*(p[j-3] + p[j+1]);
            ++j;
            o[j] = A*p[j] + B*(p[j-1] + p[j]) + C*(p[j-2] + p[j]) + D*(p[j-3] + p[j]);
        }
        
        if (i > 2) {
            uint8_t *o = out[i-3];
            for (int j=0; j<w; ++j) {
                o[j] = rounded_shift<Bits*2>(A*r[3][j] +
                                             B*(r[2][j] + r[4][j]) +
                                             C*(r[1][j] + r[5][j]) +
                                             D*(r[0][j] + r[6][j]));
            }
        }

        uint32_t *old_r0 = r[0];
        for (int k=0; k<6; ++k)
            r[k] = r[k+1];
        if (i<6)
            r[6] += w;
        else if (i+1 < in.height())
            r[6] = old_r0;
    }    
}



void ecv::compute_gaussian_kernel(double sigma, int n, unsigned int scale,
                                  int kernel[],
                                  GaussianKernel::Type type)
{
    std::vector<double> kf(n);
    double f = 1.0 / (sqrt(2)*sigma);
    double sum = 0;
    for (int i=0; i<n; ++i) {
        double y=0;
        switch (type) {
        case GaussianKernel::SAMPLED_PDF:
            y = exp(-(i*i)/(2*sigma*sigma));
            break;
        case GaussianKernel::SAMPLED_CDF:
            if (i+1<n)
                y = erf((i+0.5)*f) - erf((i-0.5)*f);
            else
                y = erfc((i-0.5)*f);
            break;
        }
        kf[i] = y;

        sum += (i == 0) ? y : 2*y;
    }
    
    double factor = scale / sum;
    int isum = 0;
    for (int i=0; i<n; ++i)
        isum += kernel[i] = (int)(kf[i]*factor + 0.5);

    isum = isum*2 - kernel[0];

    int err = isum - scale;

    kernel[0] -= err%2;
    err -= err%2;

    if (err == 0)
        return;    
    
    for (int i=1; i<n && err != 0; ++i) {
        int d = err < 0 ? 1 : -1;
        kernel[i] += d;
        err += d*2;
    }
}

