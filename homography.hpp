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

#ifndef ECV_SL3_HPP
#define ECV_SL3_HPP

#include <ecv/correspondence.hpp>
#include <latl/sl3.hpp>
#include <latl/lu.hpp>
#include <latl/ldlt.hpp>

namespace ecv {


    template <class Scalar>
    struct HomographyTest
    {
        Scalar r;
        HomographyTest(Scalar radius) : r(radius) {}
        bool operator()(const latl::SL3<Scalar>& sl3, const ScaledMatch2D<Scalar>& m) const
        {
            latl::Vector<2,Scalar> h = latl::project(sl3*unproject(m.a));
            return latl::norm_sq(m.b - h) < latl::sq(r*m.scale);
        }
    };

    struct HomographyGenerator
    {
        float max_sf_sq;
        HomographyGenerator(float max_scale = 1e20f) : max_sf_sq(max_scale*max_scale) {}
        
        template <class Corr, class Scalar>
        void operator()(const Corr *matches[4], std::vector<latl::SL3<Scalar> >& models) const
        {
            using namespace latl;

            Matrix<8,8,Scalar> D;
            Vector<8,Scalar> b;
            for (int i=0; i<4; ++i) {
                const Corr& m = *matches[i];
                D(i*2,0) = D(i*2+1,3) = m.a[0];
                D(i*2,1) = D(i*2+1,4) = m.a[1];
                D(i*2,2) = D(i*2+1,5) = 1;
                D(i*2,3) = D(i*2+1,0) = 0;
                D(i*2,4) = D(i*2+1,1) = 0;
                D(i*2,5) = D(i*2+1,2) = 0;
                D(i*2,6) = -m.b[0] * m.a[0];
                D(i*2,7) = -m.b[0] * m.a[1];
                D(i*2+1,6) = -m.b[1] * m.a[0];
                D(i*2+1,7) = -m.b[1] * m.a[1];
                b[i*2] = m.b[0];
                b[i*2+1] = m.b[1];
            }

            LU<8,Scalar> lu(D);
            Vector<8,Scalar> h = lu.inverse_times(b);
            Matrix<3,3,Scalar> H;
            H[0] = slice<0,3>(h);
            H[1] = slice<3,3>(h);
            H[2] = unproject(slice<6,2>(h));
            Scalar scale_sq = latl::determinant(slice<0,0,2,2>(H));
            if (scale_sq > max_sf_sq || scale_sq * max_sf_sq < 1)
                return;
            
            models.push_back(SL3<Scalar>::from_matrix(H));
        }
    };

    template <class Scalar>
    bool refine(const std::vector<ScaledMatch2D<Scalar> >& corr,
                const std::vector<bool>& inlier,
                latl::SL3<Scalar>& trans)
    {
        using namespace latl;

        Matrix<8,8,Scalar> A(Scalar(0));
        Vector<8,Scalar> b(Scalar(0));
        
        for (size_t i=0; i<corr.size(); ++i)
        {
            if (!inlier[i])
                continue;

            Vector<3,Scalar> y = trans * unproject(corr[i].a);
            Vector<2,Scalar> h = project(y);
            
            Vector<2,Scalar> v = corr[i].b - h;
            Scalar Rinv = 1 / sq(corr[i].scale);

            Matrix<2,8,Scalar> J;
            J(0,0) = J(1,1) = 1;
            J(0,1) = J(1,0) = 0;
            J(0,2) = -h[1];
            J(1,2) = h[0];
            J(0,3) = 3*h[0];
            J(1,3) = 3*h[1];
            J(0,4) = h[0];
            J(1,4) = -h[1];
            J(0,5) = h[1];
            J(1,5) = h[0];
            J(0,6) = -h[0]*h[0];
            J(1,6) = -h[1]*h[0];
            J(0,7) = -h[0]*h[1];
            J(1,7) = -h[1]*h[1];

            outer_product_upper(J.T(), Rinv, ops::Add(), A);
            b += J.T() * (Rinv * v);
        }
        
        LDLT<8,Scalar> ldlt(A);
        if (!ldlt.is_full_rank())
            return false;
        Vector<8,Scalar> update = ldlt.inverse_times(b);
        trans = SL3<Scalar>::exp(update) * trans;
        return true;
    }
    
    
}

#endif
