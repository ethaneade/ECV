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

#ifndef ECV_AFFINE_HPP
#define ECV_AFFINE_HPP

#include <ecv/correspondence.hpp>
#include <latl/aff2.hpp>
#include <latl/lu.hpp>
#include <latl/ldlt.hpp>

namespace ecv {


    template <class Scalar>
    struct AffineTest
    {
        Scalar r;
        AffineTest(Scalar radius) : r(radius) {}
        bool operator()(const latl::Aff2<Scalar>& aff2, const ScaledMatch2D<Scalar>& m) const
        {
            return latl::norm_sq(m.b - aff2*m.a) < latl::sq(r*m.scale);
        }
    };

    struct AffineGenerator
    {
        float max_sf_sq;
        AffineGenerator(float max_scale = 1e20f) : max_sf_sq(max_scale*max_scale) {}
        
        template <class Corr, class Scalar>
        void operator()(const Corr *matches[3], std::vector<latl::Aff2<Scalar> >& models) const
        {
            using namespace latl;

            Matrix<3,3,Scalar> D;
            Vector<3,Scalar> bx, by;
            for (int i=0; i<3; ++i) {
                D[i] = unproject(matches[i]->a);
                bx[i] = matches[i]->b[0];
                by[i] = matches[i]->b[1];
            }

            LU<3,Scalar> lu(D);
            Matrix<2,3,Scalar> At;
            At[0] = lu.inverse_times(bx);
            At[1] = lu.inverse_times(by);

            Scalar scale_sq = latl::determinant(slice<0,0,2,2>(At));
            if (scale_sq > max_sf_sq || scale_sq * max_sf_sq < 1)
                return;
            
            models.push_back(Aff2<Scalar>(At));
        }
    };

    template <class Scalar>
    bool refine(const std::vector<ScaledMatch2D<Scalar> >& corr,
                const std::vector<bool>& inlier,
                latl::Aff2<Scalar>& trans)
    {
        using namespace latl;
        Scalar s=0,sx=0,sy=0,sxy=0,sxx=0,syy=0;
        Scalar sv0x=0, sv0y=0, sv1x=0, sv1y=0, sv0=0, sv1=0;
        for (size_t i=0; i<corr.size(); ++i)
        {
            if (!inlier[i])
                continue;
            
            Vector<2,Scalar> h = trans * corr[i].a;
            Vector<2,Scalar> v = corr[i].b - h;
            Scalar Rinv = 1 / sq(corr[i].scale);

            Scalar x = h[0];
            Scalar y = h[1];
            Scalar wx = Rinv*x;
            Scalar wy = Rinv*y;
            s += Rinv;
            sx += wx;
            sy += wy;
            sxy += wx*y;
            sxx += wx*x;
            syy += wy*y;
            
            Scalar wv0 = Rinv*v[0];
            Scalar wv1 = Rinv*v[1];
            sv0 += wv0;
            sv1 += wv1;
            sv0x += wv0*x;
            sv0y += wv0*y;
            sv1x += wv1*x;
            sv1y += wv1*y;
        }
        
        Matrix<6,6,Scalar> A;
        Vector<6,Scalar> b;
        
        A(0,0) = A(1,1) = s;
        A(0,1) = A(2,3) = A(4,5) = 0;
        A(0,2) = A(1,4) = -sy;
        A(0,3) = A(0,4) = A(1,2) = A(1,5) = sx;
        A(0,5) = A(1,3) = sy;
        A(2,2) = A(3,3) = A(4,4) = A(5,5) = sxx + syy;
        A(2,5) = A(3,4) = sxx - syy;
        Scalar twoxy = 2*sxy;
        A(2,4) = -twoxy;
        A(3,5) = twoxy;
        
        b[0] = sv0;
        b[1] = sv1;
        b[2] = sv1x - sv0y;
        b[3] = sv0x + sv1y;
        b[4] = sv0x - sv1y;
        b[5] = sv0y + sv1x;
        
        LDLT<6,Scalar> ldlt(A);
        if (!ldlt.is_full_rank())
            return false;
        Vector<6,Scalar> update = ldlt.inverse_times(b);
        trans = Aff2<Scalar>::exp(update) * trans;
        return true;
    }
    
    
}

#endif
