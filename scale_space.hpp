#ifndef ECV_SCALE_SPACE_HPP
#define ECV_SCALE_SPACE_HPP

#include <ecv/image.hpp>
#include <vector>

namespace ecv {

    namespace scale_space {
        
        template <class T>
        struct Pyramid
        {
            std::vector<Image<T> > level;
            std::vector<Image<T> > diff;
            double scale_factor;
        };

        struct PyramidBuilder
        {
            PyramidBuilder();
            ~PyramidBuilder();
            
            void init(double scale_factor,
                      double preblur_sigma,
                      double assumed_blur_sigma);
            
            void compute(const Image<float>& im, int levels, Pyramid<float>& pyr);
            void compute(const Image<float>& im, Pyramid<float>& pyr, int min_dim = 20);
            struct State;
            State *state;
        };

        struct Point
        {
            float x, y;
            float scale;
            float strength;

            int level;
            float level_x, level_y;
            float level_scale;            
        };

        void find_extrema(const Pyramid<float>& pyr, float thresh,                          
                          std::vector<Point>& extrema);

        
    }
}


#endif
