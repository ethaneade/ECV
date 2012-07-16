#include <ecv/resample.hpp>
#include <cmath>

using namespace ecv;


void ecv::BicubicScaler::init(int max_dim, double scale_)
{        
    const int n = max_dim;
    coef.resize(n*4);
    base.resize(n);
            
    scale = scale_;
    double step = 1.0 / scale;
    for (int i=0; i<n; ++i) {
        double x = i*step;
        int ix = (int)x;
        base[i] = ix;
        double t = x - ix;
        double tt = t*t;
        double ttt = tt*t;
        int16_t *c = &coef[4*i];
        c[0] = (int16_t)round(16384.0 * 0.5*(-ttt + 2*tt - t));
        c[1] = (int16_t)round(16384.0 * 0.5*(3*ttt - 5*tt + 2));
        c[2] = (int16_t)round(16384.0 * 0.5*(-3*ttt + 4*tt + t));
        c[3] = (int16_t)round(16384.0 * 0.5*(ttt - tt));
    }
}

void ecv::bicubic_scale(const Image<uint8_t> &in, BicubicScaler &scaler,
                        Image<uint8_t>& out)
{
    Image<int16_t> narrow((int)(in.width() * scaler.scale + 0.5), in.height());    
    out.resize(narrow.width(), (int)(in.height() * scaler.scale + 0.5));    

    // Find horizontal edge condition bounds
    int interior_start = 0;
    while (interior_start < in.width() &&
           scaler.base_at(interior_start) == 0)
    {
        ++interior_start;
    }

    int interior_end = in.width()-1;
    while (interior_end > 0 &&
           scaler.base_at(interior_end)+2 >= in.width())
    {
        --interior_end;
    }
           
    for (int i=0; i<in.height(); ++i) {
        const uint8_t *p = in[i];
        int16_t *o = narrow[i];

        for (int j=0; j<interior_start; ++j) {
            const int16_t *c = scaler.coef_at(j);
            int y1 = p[0], y2 = p[1], y3 = p[2];
            int iy = ((c[0] + c[1])*y1 + c[2]*y2 + c[3]*y3 + (1<<7)) >> 8;
            o[j] = iy;
        }
        
        for (int j=interior_start; j<interior_end; ++j) {
            int ix = scaler.base_at(j);
            const int16_t *c = scaler.coef_at(j);

            int y0 = p[ix-1], y1 = p[ix], y2 = p[ix+1], y3 = p[ix+2];
            int iy = (c[0]*y0 + c[1]*y1 + c[2]*y2 + c[3]*y3 + (1<<7)) >> 8;
            o[j] = iy;
        }
        
        for (int j=interior_end; j<narrow.width(); ++j) {
            int ix = scaler.base_at(j);
            const int16_t *c = scaler.coef_at(j);
            int x2 = std::min(in.width()-1,ix+1);
            int y0 = p[ix-1], y1 = p[ix], y2 = p[x2];
            int iy = (c[0]*y0 + c[1]*y1 + (c[2] + c[3])*y2 + (1<<7)) >> 8;
            o[j] = iy;
        }
    }
    
    const int s = narrow.stride();
    for (int i=0; i<out.height(); ++i) {
        uint8_t *o = out[i];
        int iy = scaler.base_at(i);
        const int16_t *p = narrow[iy];
        int x0 = iy > 0 ? -s : 0;
        int x2 = iy < narrow.height()-1 ? s : 0;
        int x3 = iy < narrow.height()-2 ? s*2 : x2;

        const int16_t *c = scaler.coef_at(i);
        for (int j=0; j<out.width(); ++j, ++p) {
            int y0 = p[x0], y1 = p[0], y2 = p[x2], y3 = p[x3];
            int iz = (c[0]*y0 + c[1]*y1 + c[2]*y2 + c[3]*y3 + (1<<19)) >> 20;
            *o++ = iz < 0 ? 0 : (iz > 255 ? 255 : iz);
        }
    }
}

