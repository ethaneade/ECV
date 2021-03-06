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
#include <ecv/epipolar.hpp>

using namespace ecv;
using namespace latl;


template <class S>
S ecv::epipolar_error(const latl::Vector<2,S>& a,
                      const latl::Vector<2,S>& b,
                      const latl::SE3<S>& motion)
{
    Vector<3,S> Ra = motion.rotation() * unproject(a);
    Vector<3,S> line = motion.translation() ^ Ra;
    S scale_sq = line[0]*line[0] + line[1]*line[1];   
    return (b[0]*line[0] + b[1]*line[1] + line[2]) / latl::sqrt(scale_sq);
}


template 
float ecv::epipolar_error<float>(const latl::Vector<2,float>& a,
                                 const latl::Vector<2,float>& b,
                                 const latl::SE3<float>& motion);

template 
double ecv::epipolar_error<double>(const latl::Vector<2,double>& a,
                                   const latl::Vector<2,double>& b,
                                   const latl::SE3<double>& motion);

template <class S>
S ecv::epipolar_error(const latl::Vector<2,S>& a,
                      const latl::Vector<2,S>& b,
                      const latl::SE3<S>& motion,
                      latl::Vector<2,S> &Ja,
                      latl::Vector<2,S> &Jb,
                      latl::Vector<6,S> &Jmotion)
{
    Vector<3,S> Ra = motion.rotation() * unproject(a);
    Vector<3,S> line = motion.translation() ^ Ra;
    S l00 = line[0]*line[0];
    S l11 = line[1]*line[1];
    S scale_sq = l00 + l11;
    S scale = latl::sqrt(scale_sq);
    S inv_scale = 1/scale;
    S error = (b[0]*line[0] + b[1]*line[1] + line[2]) * inv_scale;

    S l0 = line[0] * inv_scale;
    S l1 = line[1] * inv_scale;
    
    Vector<3,S> dline;
    dline[0] = inv_scale * (b[0] - l0*error);
    dline[1] = inv_scale * (b[1] - l1*error);
    dline[2] = inv_scale;

    Jb[0] = l0;
    Jb[1] = l1;
    
    slice<0,3>(Jmotion) = Ra ^ dline;
    slice<3,3>(Jmotion) = line ^ dline;

    Vector<3,S> dline_cross_t = dline ^ motion.translation();
    Ja = slice<0,0,2,3>(motion.rotation().matrix().T()) * dline_cross_t;

    return error;
}

template 
float ecv::epipolar_error<float>(const latl::Vector<2,float>& a,
                                 const latl::Vector<2,float>& b,
                                 const latl::SE3<float>& motion,
                                 latl::Vector<2,float> &Ja,
                                 latl::Vector<2,float> &Jb,
                                 latl::Vector<6,float> &Jmotion);

template 
double ecv::epipolar_error<double>(const latl::Vector<2,double>& a,
                                 const latl::Vector<2,double>& b,
                                 const latl::SE3<double>& motion,
                                 latl::Vector<2,double> &Ja,
                                 latl::Vector<2,double> &Jb,
                                 latl::Vector<6,double> &Jmotion);
