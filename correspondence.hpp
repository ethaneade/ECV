// Copyright 2011 Ethan Eade. All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:

//    1. Redistributions of source code must retain the above
//       copyright notice, this list of conditions and the following
//       disclaimer.

//    2. Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials
//       provided with the distribution.

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

// The views and conclusions contained in the software and
// documentation are those of the authors and should not be
// interpreted as representing official policies, either expressed or
// implied, of Ethan Eade.

#ifndef ECV_CORRESPONDENCE_HPP
#define ECV_CORRESPONDENCE_HPP

#include <vector>
#include <latl/latl.hpp>
#include <cassert>

namespace ecv {


    template <class Scalar>
    struct Match2D
    {
        latl::Vector<2,Scalar> a, b;
    };    

    template <class Scalar>
    struct ScaledMatch2D
    {
        latl::Vector<2,Scalar> a, b;
        Scalar scale;
    };
    
    template <class X, class Model, class Test>
    size_t find_inliers(const std::vector<X>& corr, const Model& model, const Test& test,
                        std::vector<bool>& inlier)
    {
        inlier.resize(corr.size());
        size_t count = 0;
        for (size_t i=0; i<corr.size(); ++i) {
            if (test(model, corr[i])) {
                ++count;
                inlier[i] = true;            
            } else
                inlier[i] = false;
        }
        return count;
    }

    template <class X, class Flag>
    void remove_outliers(std::vector<X>& corr, const std::vector<Flag>& inlier)
    {
        assert(inlier.size() >= corr.size());
        size_t next = 0;
        for (size_t i=0; i<corr.size(); ++i) {
            if (inlier[i])
                corr[next++] = corr[i];
        }
        corr.resize(next);
    }
}

#endif
