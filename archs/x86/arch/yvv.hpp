#ifndef ECV_X86_YVV_HPP
#define ECV_X86_YVV_HPP

#include <ecv/image.hpp>
#include <ecv/gaussian_yvv.hpp>

namespace ecv {
    namespace arch {

#define ECV_ARCH_HAS_YVV_BLUR_VERTICAL 1
#define ECV_ARCH_HAS_YVV_BLUR_HORIZONTAL 1
        
        void yvv_blur_vertical(Image<float>& im, const YvV_Params& yvv);
        void yvv_blur_horizontal(Image<float>& im, const YvV_Params& yvv);
    }
}


#endif
