#ifndef ECV_RESAMPLE_HPP
#define ECV_RESAMPLE_HPP

#include <ecv/image.hpp>
#include <stdint.h>
#include <vector>

namespace ecv {

    struct BicubicScaler
    {
        double scale;
        std::vector<int16_t> coef;
        std::vector<int> base;

        int base_at(int i) const { return base[i]; }
        const int16_t *coef_at(int i) const { return &coef[i*4]; }
        
        void init(int max_dim, double scale_);

        BicubicScaler() {}                      
        BicubicScaler(int max_dim, double s)
        {
            init(max_dim, s);
        }
    };

    void bicubic_scale(const Image<uint8_t> &in,
                       BicubicScaler &scaler,
                       Image<uint8_t>& out);
    
}

#endif
