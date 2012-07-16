#ifndef ECV_CONVOLUTION_HPP
#define ECV_CONVOLUTION_HPP

#include <ecv/image.hpp>
#include <stdint.h>

namespace ecv {    

    void convolve_121(const Image<uint8_t> &in, Image<uint8_t>& out);
    
    void convolve_symmetric_3tap_12bit(const Image<uint8_t> &in,
                                       int kernel[2],
                                       Image<uint8_t> &out);

    void convolve_symmetric_5tap_12bit(const Image<uint8_t> &in,
                                       int kernel[3],
                                       Image<uint8_t> &out);
    
    void convolve_symmetric_5tap_12bit_8to12(const Image<uint8_t> &in,
                                             int kernel[3],
                                             Image<uint16_t> &out);
    
    void convolve_symmetric_7tap_12bit(const Image<uint8_t> &in,
                                       int kernel[4],
                                       Image<uint8_t> &out);

    struct GaussianKernel
    {
        enum Type { SAMPLED_PDF,
                    SAMPLED_CDF,
        };
               
    };
    
    void compute_gaussian_kernel(double sigma, int n, unsigned int scale,
                                 int kernel[],
                                 GaussianKernel::Type type=GaussianKernel::SAMPLED_PDF);
    
}

#endif
