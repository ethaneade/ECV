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

#ifndef ECV_SIMILARITY_HPP
#define ECV_SIMILARITY_HPP

#include <ecv/correspondence.hpp>
#include <latl/sim2.hpp>
#include <latl/ldlt.hpp>

namespace ecv {


    template <class Scalar>
    struct SimilarityTest
    {
        Scalar r;
        SimilarityTest(Scalar radius) : r(radius) {}
        bool operator()(const latl::Sim2<Scalar>& sim2, const ScaledMatch2D<Scalar>& m) const
        {
            return latl::norm_sq(m.b - sim2*m.a) < latl::sq(r*m.scale);
        }
    };

    struct SimilarityGenerator
    {
        float max_scale_factor;
        SimilarityGenerator(float max_scale_factor_ = 1e20f) : max_scale_factor(max_scale_factor_) {}
        
        template <class Corr, class Scalar>
        void operator()(const Corr *matches[2], std::vector<latl::Sim2<Scalar> >& models) const
        {
            using namespace latl;
        
            const Vector<2,Scalar> da = matches[1]->a - matches[0]->a;
            const Vector<2,Scalar> db = matches[1]->b - matches[0]->b;

            Scalar da2 = norm_sq(da), db2 = norm_sq(db);
            if (da2 == 0 || db2 == 0)
                return;

            if (da2 * max_scale_factor < db2 || db2 * max_scale_factor < da2)
                return;
            
            Scalar sa = latl::sqrt(da2);
            Scalar sb = latl::sqrt(db2);
        
            Scalar s = sb / sa;
            Scalar theta = latl::acos(da * db / (sa * sb));
            Sim2<Scalar> sim2((Uninitialized()));
            sim2.scale() = s;
            sim2.rotation() = SO2<Scalar>::exp(theta);
            sim2.translation() = matches[0]->b/s - sim2.rotation()*matches[0]->a;
            models.push_back(sim2);
        }
    };

    template <class Scalar>
    bool refine(const std::vector<ScaledMatch2D<Scalar> >& corr,
                const std::vector<bool>& inlier,
                latl::Sim2<Scalar>& sim2)
    {
        using namespace latl;
        Matrix<4,4,Scalar> A(Scalar(0));
        Vector<4,Scalar> b(Scalar(0));

        for (size_t i=0; i<corr.size(); ++i)
        {
            if (!inlier[i])
                continue;
        
            Vector<2,Scalar> h = sim2 * corr[i].a;
            Vector<2,Scalar> v = corr[i].b - h;
            Matrix<2,4,Scalar> J;
            J(0,0) = J(1,1) = 1;
            J(0,1) = J(1,0) = 0;
            J(0,2) = -h[1];
            J(1,2) = h[0];
            J.T()[3] = h;

            Scalar Rinv = 1 / sq(corr[i].scale);
            A += Rinv * (J.T() * J);
            b += J.T() * (Rinv * v);
        }
        LDLT<4,Scalar> ldlt(A);
        if (!ldlt.is_full_rank())
            return false;
        Vector<4,Scalar> update = ldlt.inverse_times(b);
        sim2 = Sim2<Scalar>::exp(update) * sim2;
        return true;
    }
    
    
}

#endif
