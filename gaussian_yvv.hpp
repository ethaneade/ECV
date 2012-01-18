#include <ecv/image.hpp>

namespace ecv {
    
    struct YvV_Params
    {
        double sigma;
        float b0, b1, b2;
        float alpha;
        float m[9];

        void init(double sigma);
    };

    void yvv_blur_horizontal(Image<float>& im, const YvV_Params& yvv);
    void yvv_blur_vertical(Image<float>& im, const YvV_Params& yvv);

    void yvv_blur(Image<float>& im, const YvV_Params& yvv);
    void yvv_blur(Image<float>& im, double sigma);
}
