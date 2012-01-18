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
#ifndef ECV_CAMERA_MODEL_HPP
#define ECV_CAMERA_MODEL_HPP

#include <latl/latl.hpp>

namespace ecv {

    class CameraModel
    {
    public:
        virtual ~CameraModel() {}

        virtual const char* name() const = 0;
        
        virtual int num_params() const = 0;
        virtual latl::Vector<-1,float> get_params() const = 0;
        virtual void set_params(const latl::Vector<-1,float>& p) = 0;
        virtual void update_params(const latl::Vector<-1,float>& dp) = 0;
        
        latl::Vector<2,float> project(const latl::Vector<2,float>& xy) const
        {
            return project(xy, 0, 0);
        }

        latl::Vector<2,float> project(const latl::Vector<2,float>& xy,
                                      latl::Matrix<2,2,float>& J_xy) const
        {
            return project(xy, &J_xy, 0);
        }

        latl::Vector<2,float> project(const latl::Vector<2,float>& xy,
                                      latl::Matrix<2,2,float>& J_xy,
                                      latl::Matrix<2,-1,float>& J_model) const
        {
            return project(xy, &J_xy, &J_model);
        }

        latl::Vector<2,float> unproject(const latl::Vector<2,float>& uv) const
        {
            return unproject(uv, 0);
        }

        latl::Vector<2,float> unproject(const latl::Vector<2,float>& uv,
                                        latl::Matrix<2,2,float>& J_uv) const
        {
            return unproject(uv, &J_uv);
        }
        
    protected:
        virtual latl::Vector<2,float> project(const latl::Vector<2,float>& xy,
                                              latl::Matrix<2,2,float>* J_xy,
                                              latl::Matrix<2,-1,float>* J_model) const = 0;

        virtual latl::Vector<2,float> unproject(const latl::Vector<2,float>& uv,
                                                latl::Matrix<2,2,float>* J_uv) const = 0;
    };
    
}

#endif
