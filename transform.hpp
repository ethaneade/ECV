#ifndef ECV_TRANSFORM_HPP
#define ECV_TRANSFORM_HPP

#include <ecv/image.hpp>
#include <latl/latl.hpp>

namespace ecv {

    template <class T>
    float bilinear_sample(const Image<T>& im, float x, float y)
    {
        int lx = (int)x;
        int ly = (int)y;
        x -= lx;
        y -= ly;

        const T *base = im[ly] + lx;
        float a = base[0], b = base[1];
        float c = base[im.stride()], d = base[im.stride()+1];
        float top = a + (b-a) * x;
        float bot = c + (d-c) * x;
        return top + (bot-top) * y;
    }
    
    template <class T>
    bool warp_affine(const Image<T>& src, const latl::Matrix<2,3,float>& dst_to_src,
                     Image<T>& dst)
    {
        using namespace latl;
        const Vector<2,float> dx = dst_to_src.T()[0];
        const Vector<2,float> dy = dst_to_src.T()[1];
        const Vector<2,float> ul = dst_to_src.T()[2];
        {
            float max_x = src.width() - 1;
            float max_y = src.height() - 1;

            if (ul[0] < 0 || ul[0] >= max_x || ul[1] < 0 || ul[1] >= max_y)
                return false;
            
            Vector<2,float> ur = ul + dx * (dst.width()-1);
            if (ur[0] < 0 || ur[0] >= max_x || ur[1] < 0 || ur[1] >= max_y)
                return false;
            
            Vector<2,float> ll = ul + dy * (dst.height()-1);
            if (ll[0] < 0 || ll[0] >= max_x || ll[1] < 0 || ll[1] >= max_y)
                return false;
            
            Vector<2,float> lr = ll + dx * (dst.width()-1);
            if (lr[0] < 0 || lr[0] >= max_x || lr[1] < 0 || lr[1] >= max_y)
                return false;
        }

        for (int i=0; i<dst.height(); ++i) {
            Vector<2,float> src_xy = ul + i * dy;
            T* o = dst[i];
            for (int j=0; j<dst.width(); ++j, src_xy += dx) {
                o[j] = bilinear_sample(src, src_xy[0], src_xy[1]);                
            }
        }
        return true;
    }

    template <class T>
    bool warp_affine(const Image<T>& src,
                     const latl::Vector<2,float>& src_center,
                     const latl::Vector<2,float>& dst_center,
                     const latl::Matrix<2,2,float>& dst_to_src,
                     Image<T>& dst)
    {
        using namespace latl;
        Matrix<2,3,float> aff;
        slice<0,0,2,2>(aff) = dst_to_src;
        aff.T()[2] = src_center - dst_to_src * dst_center;
        return warp_affine(src, aff, dst);
    }

    
}

#endif
