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
