#ifndef ECV_ROBUSTIFIER_HPP
#define ECV_ROBUSTIFIER_HPP

#include <cmath>
#include <algorithm>

namespace ecv {
    
    class CostFunction
    {
    public:
        virtual float weight(float err_sq) const = 0;
        virtual float cost(float err_sq) const = 0;    
    };

    class CauchyCost : public CostFunction
    {
        float inv_c_sq;
    public:
        CauchyCost(float c=2.3849f) : inv_c_sq(1.f/(c*c)){}

        float weight(float err_sq) const {
            return 1.f/(1.f + err_sq*inv_c_sq);
        }

        float cost(float err_sq) const {
            return logf(1.f  + err_sq*inv_c_sq);
        }
    };

    struct HuberCost : public CostFunction
    {
        float k;
    public:
        HuberCost(float k_=1.21f) : k(k_){}

        float weight(float err_sq) const {
            return err_sq < k*k ? 1.f : k / sqrtf(err_sq);
        }

        float cost(float err_sq) const {
            return err_sq * weight(err_sq);
        }
    }; 

    class TukeyCost : public CostFunction
    {
        float inv_c_sq;
    public:
        TukeyCost(float c=4.681f) : inv_c_sq(1.f/(c*c)){}

        float weight(float err_sq) const {
            float y = std::max(0.f, 1.f - err_sq*inv_c_sq);
            return y*y;
        }

        float cost(float err_sq) const {
            float y = std::max(0.f, 1.f - err_sq*inv_c_sq);
            return 1.f - y*y*y;
        }
    };

}

#endif
